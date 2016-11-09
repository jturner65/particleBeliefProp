/*
* Copyright (c) 2015, Georgia Tech Research Corporation
* All rights reserved.
*
* Author(s): John Turner <jturner65@gatech.edu>
*
* Georgia Tech Graphics Lab and Humanoid Robotics Lab
*
* Directed by Prof. C. Karen Liu and Prof. Mike Stilman
* <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
*
* This file is provided under the following "BSD-style" License:
*   Redistribution and use in source and binary forms, with or
*   without modification, are permitted provided that the following
*   conditions are met:
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
*   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
*   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
*   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
*   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
*   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
*   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*/
#include "SimContext.h"
#include "BalanceMotionGoal.h"
#include "CPBPHandler.h"
#include "dart/dynamics/BodyNode.h"

using namespace std;
namespace cPBPropApp {
	BalanceMotionGoal::BalanceMotionGoal(int _numSubG, const vector<int>& _ubRbIdx) : MotionGoal(_numSubG, _ubRbIdx) {
		name = "Balance Goal";
		//(int idx, double _mean, double _sd, bool initState, bool useResForVar, const std::string& name) {
		initSubGoal(idxCOMCOP, 0, .005, false, true, "COM-COP Proj");
		initSubGoal(idxCOMDisp, 0, .005, true, true, "COM Displacement from line between feet");
		initSubGoal(idxCOMFwdVel, 0, .005, true, true, "COM Forward Velocity");
		initSubGoal(idxCOMLatVel, 0, .005, true, true, "COM Lateral Velocity");
		initSubGoal(idxHeadRelCOM, 0, .005, true, true, "Head Position Relative to COM");
		//initSubGoal(idxFootHeight,0,0.1,false,"Foot Height");
		initSubGoal(idxFootDist, 0, .005, true, true, "Foot Spread vs. Rest");
		initSubGoal(idxFootMaxDist, 0, .005, true, true, "Foot Max Spread/Stride");
		initSubGoal(idxFeetNoCross, 0, .005, false, true, "No Leg Crossing");

		initSubGoal(idxUp, 0, .005, true, true, "Hip Oriented Upright");
		initSubGoal(idxHipY, 0, .005, true, true, "Hip Height");
		initSubGoal(idxHipAng, 0, .01, false, true, "Hip Ang Vel");
		initSubGoal(idxReaching, 0, .05, false, false, "Reaching toward target");

		initSubGoal(idxRestPose, 0, .2, false, true, "Match Entire Rest Pose - Not Calced");
		initSubGoal(idxRestUpperBody, 0, .05, false, true, "Match Upper Body Rest Pose - Not Calced");
		updateSubGWtMultSds(true);			//call whenever weights or sds change - pass false if # of subgoals isn't changing
	}
	//source sim context for  
	void BalanceMotionGoal::init() {
		MotionGoal::init();

	
		updateSubGWtMultSds(false);			//call whenever weights or sds change - pass false if # of subgoals isn't changing
	}

	void BalanceMotionGoal::refresh() {
		getValsFromUI();
		MotionGoal::refresh();
		updateSubGWtMultSds(false);			//call whenever weights or sds change - pass false if # of subgoals isn't changing
		cout << "Refresh called : " << name << "\n";
	}

	//calculate the cost for varying from balance
	//timeSec is time elapsed in prediction phase (k*deltaT)
	double BalanceMotionGoal::calcFrmCost(std::shared_ptr<SimContext> simCtxt, double& varLevel, double timeSec) {
		double result = 0, resultVar = 0;
		//current post for this simcntxt - don't use pose/dof vel for now
		//Eigen::VectorXd
		//	currState = simCtxt->currState, currPoseUpper = restPoseUpper,
		//	currPose = currState.head(simCtxt->numDofs),
		//	currVel = currState.tail(simCtxt->numDofs);
		//int idx = 0;
		//for (std::vector<int>::iterator it = ubIdxs.begin(); it != ubIdxs.end(); ++it) {
		//	currPoseUpper[idx++]=currPose[*it];
		//}
		Eigen::Vector3d
			leftFootLoc = simCtxt->nodes[simCtxt->cp->lftHeelRBIdx]->getCOM(), rightFootLoc = simCtxt->nodes[simCtxt->cp->rtHeelRBIdx]->getCOM(),
			leftToeLoc = simCtxt->nodes[simCtxt->cp->lftToeRBIdx]->getCOM(), rightToeLoc = simCtxt->nodes[simCtxt->cp->rtToeRBIdx]->getCOM(),
			leftFtBallLoc = .5*(leftFootLoc + leftToeLoc), rightFtBallLoc = .5*(rightFootLoc + rightToeLoc),

			leftHandLoc = simCtxt->nodes[simCtxt->cp->lftHandRBIdx]->getCOM(), rightHandLoc = simCtxt->nodes[simCtxt->cp->rtHandRBIdx]->getCOM(),
			
			hipLoc = simCtxt->nodes[simCtxt->cp->pelvisRBIdx]->getCOM(),
			headLoc = simCtxt->nodes[simCtxt->cp->headRBIdx]->getCOM(),				//head com
			headRelCOM = headLoc - simCtxt->COM,
			bdyUp = simCtxt->PBPState.head<3>(),
			COP_COM = simCtxt->COM - simCtxt->COP;
		
		COP_COM.normalize();

		Eigen::Vector3d
			lftPt = leftFootLoc,					rftPt = rightFootLoc;
		lftPt(1) = gnd_loc(1);						rftPt(1) = gnd_loc(1);						//doing this to measure distance from feet in x-z plane
		Eigen::Vector3d
			comInPlane = simCtxt->COM,
			clsSptFt = clsPntOnSegWithBffr(simCtxt->COM, lftPt, rftPt, .02),						//penalize com deviation from line connecting balls of feet - this is closest point
			lftRelCOM = lftPt - simCtxt->COM,		rftRelCOM = rftPt - simCtxt->COM;

		lftRelCOM(1) = rp_Fwd(1);					rftRelCOM(1) = rp_Fwd(1);					//set y coord
		comInPlane(1) = gnd_loc(1);					clsSptFt(1) = gnd_loc(1);							//these two coplanar to only measure deviation in x-z plane
		lftRelCOM.normalize();						rftRelCOM.normalize();						//unit vectors in dir of feet from com
		Eigen::Vector3d
			lftCrossFwd = rp_Fwd.cross(lftRelCOM),	rftCrossFwd = rftRelCOM.cross(rp_Fwd);		//to keep feet from crossing - penalize any y value negative in either of these - need to recalc and use hip forward instead of rp_fwd

		//if (-1 == simCtxt->sampleIDX) {
		//	cout << "Com in plane : " << buildStrFromEigen3d(comInPlane) << " | closest point on line between feet : " << buildStrFromEigen3d(clsSptFt) << " | left foot point " << buildStrFromEigen3d(lftPt) << " | right foot point" << buildStrFromEigen3d(rftPt) << "\n";
		//}

		int gIdx;
		//eventually put this together as list of function pointers
		vector<double> calcVals(numSubGoals);
		calcVals[idxCOMCOP] =		(!subGFlags[idxCOMCOP]) ?  0 : calcProjSq(COP_COM, upVec);						//deviation of com projected on ground from cop
		calcVals[idxCOMDisp] =		(!subGFlags[idxCOMDisp]) ? 0 : (comInPlane - clsSptFt).norm();					//deviation of com from closest point to line projected between feet
		calcVals[idxCOMFwdVel] =	(!subGFlags[idxCOMFwdVel]) ? 0 : simCtxt->COMLinVel.dot(rp_Fwd);				//com velocity in direction of forward vector
		calcVals[idxCOMLatVel] =	(!subGFlags[idxCOMLatVel]) ? 0 : simCtxt->COMLinVel.dot(rp_Rght);			//abs of com vel in lateral dir
		//calcVals[idxHeadRelCOM] = (headRelCOM - rHeadRelCOM).norm();			//length of difference of vectors of com to head

		//calcVals[idxHeadRelCOM] = headRelCOM.cross(rHeadRelCOM).norm();			//length of difference of vectors of com to head
		calcVals[idxHeadRelCOM] = (!subGFlags[idxHeadRelCOM]) ? 0 : rHeadRelCOM.cross(headRelCOM).norm();			//length of difference of vectors of com to head
		if (calcVals[idxHeadRelCOM] < 0) {
			cout << "Head rel com in xprod < 0 : rest headRel Com " << buildStrFromEigen3d(rHeadRelCOM) << " | head rel coom : " << buildStrFromEigen3d(headRelCOM) << "\n";// << " | left foot point " << buildStrFromEigen3d(lftPt) << " | right foot point" << buildStrFromEigen3d(rftPt) << "\n";

		}
		//calcVals[idxFootHeight] = avgFootHeight;
		//foot dist also needs to take into account spread - currently feet can try to align toe to heel
		double footDist = (leftFtBallLoc - rightFtBallLoc).norm() - rFootWdth;
		calcVals[idxFootDist] =		(!subGFlags[idxFootDist]) ? 0 : footDist;
		calcVals[idxFootMaxDist] =	(!subGFlags[idxFootMaxDist]) ? 0 : (footDist > maxLegDist ? footDist - maxLegDist : 0);
		calcVals[idxFeetNoCross] =	(!subGFlags[idxFeetNoCross]) ? 0 : (((lftCrossFwd(1) < 0) ? -1 * lftCrossFwd(1) : 0) + ((rftCrossFwd(1) < 0) ? -1 * rftCrossFwd(1) : 0));
		calcVals[idxUp] =			(!subGFlags[idxUp]) ? 0 : (bdyUp - upVec).norm();
		calcVals[idxHipY] =			(!subGFlags[idxHipY]) ? 0 : (hipLoc(1) - gnd_loc(1)) - rHipHght;
		calcVals[idxHipAng] =		(!subGFlags[idxHipAng]) ? 0 : simCtxt->nodes[0]->getAngularVelocity().norm();


		calcVals[idxReaching] =		((reachHandIDX < 0) || (!subGFlags[idxReaching])) ? 0 : abs((simCtxt->nodes[reachHandIDX]->getCOM() - reachHandTarget).norm());
		
		//calcVals[idxRestPose] = (currPose - restPose).norm();
		//calcVals[idxRestUpperBody] =(currPoseUpper - restPoseUpper).norm();
		calcVals[idxRestPose] = 0;
		calcVals[idxRestUpperBody] = 0;

		Eigen::VectorXd resVals(numSubGoals), 
			resValsVar(numSubGoals);					//result values that contribute to variance - do not include reaching cost
		if (SM[useSqCosts]) {	
			for (int i = 0; i < numSubGoals; ++i) {			
				resVals[i] = getSqCostCalc(i, calcVals[i]);	
				resValsVar[i] = subGUseVar[i] ? resVals[i] : 0;				
			}
		}
		else {					
			for (int i = 0; i < numSubGoals; ++i) {			
				resVals[i] = getCostCalc(i, calcVals[i]);
				resValsVar[i] = subGUseVar[i] ? resVals[i] : 0;
			}
		}

		result = costMult * resVals.sum();
		resultVar = costMult * resValsVar.sum();

		if (-1 == simCtxt->sampleIDX) {//show subgoal costs for main walker
			for (int i = 0; i < numSubGoals; ++i) {
				int capIdx = (i * 4);
				UI->MyPgUICaption[capIdx]->label = UI->MyPgUICaption[capIdx]->buildLabel(true, (float)(resVals[i]), "%.4f");
			}
		}
		varLevel = calcVarLevel(resultVar);
		return result;
	}
	//use current pose of passed skeleton in simCtxt as rest pose target for this goal 
	void BalanceMotionGoal::setRestState(std::shared_ptr<SimContext> _srcSimCtxt) {
		MotionGoal::setRestState(_srcSimCtxt);
		init();//to reset means
	}
	//use current pose of passed skeleton in simCtxt as target pose for this goal 
	void BalanceMotionGoal::setTargetState(std::shared_ptr<SimContext> simCtxt) {
		MotionGoal::setTargetState(simCtxt);
	}

	void BalanceMotionGoal::setSubGFlag(int idx, bool val, bool useResInVar) {
		subGFlags[idx] = val;
		subGUseVar[idx] = useResInVar;
		switch (idx) {
			case idxCOMCOP : {break;}				//com-cop distance projected on a plane
			case idxCOMDisp : {break;}				//COM displacement from line between balls of feet
			case idxCOMFwdVel : {break;}			//fwd COM vel 
			case idxCOMLatVel : {break;}			//lat COM vel
			case idxHeadRelCOM : {break;}			//penalize head deviating position relative to COM
			//case idxFootHeight : {break;}			//
			case idxFootDist : {break;}				//foot separation that deviates from a certain value
			case idxFootMaxDist : {break;}			//penalize splits and giant strides - anything beyond a max distance
			case idxUp : {break;}					//hipOrientation
			case idxHipY : {break;}					//hip height
			case idxHipAng : {break;}				//hip angular velocity(twisting)
			case idxReaching: {break; }				//reaching with selected hand
			case idxRestPose : {break;}				//closeness to joint angles of the rest pose
			case idxRestUpperBody : {break;}		//closeness to joint angles of the upper body of the rest pose
		}
	}//setSubGFlag


	vector<double> BalanceMotionGoal::accumulateVals() {		//grab all locals that are modifiable by UI or used elsewhere
		vector<double> res = MotionGoal::accumulateVals();
		//add goal specific data here
		return res;
	}

	void BalanceMotionGoal::distributeVals(vector<double>& vals) {//copy all UI values to their appropriate lcl variables (from UI or file)
		MotionGoal::distributeVals(vals);							//main weights, means and sds are set in MotionGoal::distributeVals
	}

	//get values to send to UI
	void BalanceMotionGoal::sendValsToUI() {
		MotionGoal::sendValsToUI();
		//vector<double> res = accumulateVals();
		//for (int i = 0; i < UI->MyPgUISldr.size(); ++i) { UI->MyPgUISldr[i]->setSliderVal((float)(res[i])); }
	}

	void BalanceMotionGoal::resetValues(bool dummy) {
		MotionGoal::resetValues(dummy);	
		init();
		for (int i = 0; i < numSubGoals; ++i) { UI->MyPgUIBtn[i + 3]->isChecked = subGFlags[i]; }//update UI flags	
	}//set some reasonable defaults in here that we can return to

	void BalanceMotionGoal::getValsFromUI() { MotionGoal::getValsFromUI();	 }
	void BalanceMotionGoal::resetUIWithDefVals() {	resetValues(true);	sendValsToUI();	}
	void BalanceMotionGoal::saveUIVals() {	cout << "Not implemented yet" << endl;}
	void BalanceMotionGoal::buildUI() {	MotionGoal::buildUI();	}

	bool BalanceMotionGoal::handleBtnClick() {  //only for goal-specific buttons
		bool res = MotionGoal::handleBtnClick();
		if (res) { return res; }			//if found in base-class-related buttons, return true
		for (int i = 0; i < UI->MyPgUIBtn.size(); i++) {		//go through buttons again to see if there are any balance-motion-goal-specific buttons that have been clicked
			if (UI->MyPgUIBtn[i]->isClicked()) {
				std::string onclick = UI->MyPgUIBtn[i]->getOnClick();
				cout << "Goal : " << name << " class-specific Button Clicked ID:" << i << "|" << UI->MyPgUIBtn[i]->getLabel() << " onclick = " << onclick << endl;
				switch (i) {					
					default: {return false; }
				}
			}//if click
		}//for each button
		return false;
	}

	std::ostream& operator<<(std::ostream& out, BalanceMotionGoal& mg) {
		out << (MotionGoal&)mg;

		return out;
	}
}