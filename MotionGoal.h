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

#ifndef APPS_PARTICLEBELIEFPROP_MotionGoal_H_
#define APPS_PARTICLEBELIEFPROP_MotionGoal_H_

#include <vector>
#include <memory>
#include <string>
#include <Eigen/Dense>
#include "MyGuiHandler.h"
namespace cPBPropApp {
	class SimContext;

	/// base class for goals of motion
	class MotionGoal {
	public:
		MotionGoal(int _numSubG, const vector<int>& _ubRbIdx) : SM(0), name("MotionGoal"), restState(), targetState(), lowVarLvlCost(10), highVarLvlCost(400), 
			numSubGoals(_numSubG), fullPoseSd(.1), UI(nullptr), rCOMHght(), rHipHght(), maxLegDist(.8), reachHandIDX(-1), reachHandTarget(0,0,0),
			subGWts(), subGMean(), subGSds(), subGNames(), subGWtDivSds(), costMult(1),
			rFootWdth(.187), upVec(0, 1, 0), rp_Up(0, 1, 0), rp_Fwd(1, 0, 0), rp_Rght(0, 0, 1),
			gnd_loc(0, -1, 0), ubIdxs(_ubRbIdx), restPoseUpper()
		
		{
			//int tmpUbiIdxs[13] = { 12, 13, 16, 21, 22, 23, 24, 27, 28, 29, 30, 31, 32 };			//OLD dof idx's of upper body values - don't care about elbow and wrist dofs
			//int tmpUbiIdxs[13] = { 20, 21, 22, 23, 24, 25, 26, 27, 28, 31, 32, 33, 34};			//dof idx's of upper body values - don't care about elbow and wrist dofs : 29, 30, 35, 36 
			//ubIdxs.insert(ubIdxs.end(), &tmpUbiIdxs[0], &tmpUbiIdxs[13]);
			
			restPoseUpper.setZero(13); 
			newLowVarLvlCost = lowVarLvlCost;
			newHiVarLvlCost = highVarLvlCost;
			SM.resize(numSMFlags);
			for (int i = 0; i < numSMFlags; ++i) { SM[i] = false; }
			SM[useSqCosts] = true;									//use squared goals by default
			SM[showUI] = true;										//show UI by default
			subGFlags.resize(numSubGoals, true);
			subGUseVar.resize(numSubGoals, true);
			subGWts.setConstant(numSubGoals, 1.0 / numSubGoals);					//all goals initially weighted equally
			subGMean.setConstant(numSubGoals, 0);								//means of all goals' desired distances from target
			subGSds.setConstant(numSubGoals, 1);								//all goals initially have sd = 1
			subGWtDivSds.setConstant(numSubGoals, 1);
			subGWtDivSqSds.setConstant(numSubGoals, 1);
			subGNames.resize(numSubGoals, "");
		}

		virtual ~MotionGoal() {		}

		virtual void init();
		virtual void refresh();
		virtual double calcFrmCost(std::shared_ptr<SimContext> simCtxt, double& varLevel, double timeSec) { return 0; }
		virtual void setRestState(std::shared_ptr<SimContext> simCtxt);
		virtual void setTargetState(std::shared_ptr<SimContext> simCtxt);
		virtual inline void refreshGoal() {
			if (SM[updVarLvlStp]) {
				lowVarLvlCost = newLowVarLvlCost;
				highVarLvlCost = newHiVarLvlCost;
				newLowVarLvlCost = 100000000;
				newHiVarLvlCost = 0;
				sendValsToUI();
			}
		}				//call at start of every forward sim

		void setReachHand(int rhndIdx) { reachHandIDX = rhndIdx; }
		void setReachTarget(const Eigen::Vector3d& rchTar) { reachHandTarget = rchTar; }
		void resetReach() { reachHandIDX = -1; reachHandTarget << 0, 0, 0; }

		void setFwdVec(const Eigen::Vector3d &fwd);
		void setSMBool(int idx, bool val);
		void setSMBools(std::vector<int>& idxs, std::vector<bool>& vals) { for (int i = 0; i < idxs.size(); ++i) { setSMBool(idxs[i], vals[i]); } }
		virtual void setSubGFlag(int idx, bool val, bool useResInVar) = 0;
		void setSubGFlags(std::vector<int>& idxs, std::vector<bool>& vals, std::vector<bool>& useResVals) { for (int i = 0; i < idxs.size(); ++i) { setSubGFlag(idxs[i], vals[i], useResVals[i]); } }

		//build the UI construct to modify the values of this class
		//virtual void initUIObj(int idx, string label, int type, vector<float>& vals, vector<int>& objType, vector<string>& objLabels, vector<vector<int>>& objXY_WH, vector<vector<float>>& objClr);
		virtual void buildUI();
		virtual void sendValsToUI();
		virtual void initUI() = 0;
		virtual void resetValues(bool sendToCntxts);
		virtual void getValsFromUI();
		virtual vector<double> accumulateVals();
		virtual void distributeVals(vector<double>& vals);
		virtual void resetUIWithDefVals();
		virtual void saveUIVals();
		//event handler for UI components related to this class
		virtual bool handleBtnClick();

		inline void initSubGoal(int idx, double _mean, double _sd, bool initState, bool useResInVar, const std::string& name) {
			subGMean(idx) = _mean; 
			subGSds(idx) = _sd;
			subGFlags[idx] = initState;
			subGUseVar[idx] = useResInVar;
			subGNames[idx] = std::string(name);
		}

		//if low and high variance levels change, modify
		void resetVarLvlSldrVals(double _lowVarLvl, double _hiVarLvl) {
			int highSldr = UI->MyPgUISldr.size() - 1, loSlidr = highSldr - 1;
			float fLowVal = (float)(_lowVarLvl), fHiVal = (float)(_hiVarLvl);
			UI->MyPgUISldr[loSlidr]->setSliderVal(fLowVal);
			UI->MyPgUISldr[highSldr]->setSliderVal(fHiVal);
		}

		//find closest point on segment between two points, with a buffer zone around the segment
		Eigen::Vector3d clsPntOnSegWithBffr(const Eigen::Vector3d& pt, const Eigen::Vector3d& segP1, const Eigen::Vector3d& segP2, double bufferZone) {
			Eigen::Vector3d v = segP2 - segP1,
				dir = v.normalized(),
				middle = 0.5 * (segP1 + segP2);
			double midProj = (middle - segP1).dot(dir), bufAmt = (midProj < bufferZone ? midProj : bufferZone);
			Eigen::Vector3d buffP1 = segP1 + dir * bufAmt, buffP2 = segP2 - dir * bufAmt;
			v = buffP2 - buffP1;
			double proj = (pt - buffP1).dot(dir);
			if ((proj > 0) && (proj < v.norm())) { return buffP1 + proj * dir; }
			else if (proj <= 0) { return buffP1; }
			return buffP2;
		}

		inline double getSqCostCalc(int idx, double val) {
			//if (!subGFlags[idx]) { return 0; }
			double diff = (val - subGMean(idx));
			double cost = subGWtDivSqSds(idx) * diff * diff;
			if (isnan(cost)) {
				cout << "Nan cost for eval " << subGNames[idx] << " with value : " << val << endl;			
				cost = 100000000.0;// highVarLvlCost;
			}
			return cost;
		}

		inline double getCostCalc(int idx, double val) {
			//if (!subGFlags[idx]) { return 0; }
			double cost = subGWtDivSds[idx]  * abs(val - subGMean(idx));
			if (isnan(cost)) {
				cout << "Nan cost for eval " << subGNames[idx] << " with value : " << val << endl;			
				cost = 100000000.0;// highVarLvlCost;
			}
			return cost;
		}

		inline double calcVarLevel(double costRes) {
			//varLvlCosts denote low and high variance costs
			double varTmp = ((costRes - lowVarLvlCost) / (highVarLvlCost - lowVarLvlCost));
			//set bounds on variance cap for next round of prediction iterations based on max and min results this time around
			if (SM[updVarLvlStp]) {
				newLowVarLvlCost = (costRes < newLowVarLvlCost ? costRes : newLowVarLvlCost);
				newHiVarLvlCost = (costRes > newHiVarLvlCost ? costRes : newHiVarLvlCost);
			}
			//double varRes = (varTmp < 0.000000001 ? 0.000000001 : (varTmp > 1 ? 1 : varTmp));
			double varRes = (varTmp < 0 ? 0 : (varTmp > 1 ? 1 : varTmp));
			return varRes;
		}

		inline double calcProjSq(const Eigen::Vector3d& a, const Eigen::Vector3d& n) {
			double res = (a.cross(n)).squaredNorm();
			return res;
		}//sqrd length of vector projected on the plane described by normal n
		inline double calcProj(const Eigen::Vector3d& a, const Eigen::Vector3d& n) { return (a.cross(n)).norm(); }//sqrd length of vector projected on the plane described by normal n
		inline double calcSqDist(const Eigen::Vector3d& a, const Eigen::Vector3d& b) { return (a - b).squaredNorm(); }
		inline double errf(double val) { return val*val; }
		inline double errv(Eigen::Vector3d& val) { return val.squaredNorm(); }
		//recalculates the vector of pre-calculated weights over means, so that cost can be calculated as dot of wtdifsds with (calcVals - means)
		void updateSubGWtMultSds(bool reDist);

		inline string buildStrFromDbl(double val, const char* fmt = "%.4f") {
			char buf[MAX_BUF];
			stringstream ss;
			sprintf(buf, fmt, val);
			ss << buf;
			return ss.str();// label = ss.str();
		}//buildLabel
		std::string buildStrFromEigen3d(const Eigen::Ref<const Eigen::Vector3d>& vec) { stringstream ss;  ss << buildStrFromDbl(vec(0)) << "," << buildStrFromDbl(vec(1)) << "," << buildStrFromDbl(vec(2)); return ss.str(); }

		friend std::ostream& operator<<(std::ostream& out, MotionGoal& goal);

	public: //variables
		std::string name;
		Eigen::VectorXd restState,						//the rest pose of the skeleton - set from master context on init, use for various comparisons
			restPose,									//target rest pose
			restPoseUpper,								//target rest pose upper body only (non-upper body vals are 0)
			restDofVel,									//target rest velocity of all dofs
			targetState,								//specific target state to match
			targetPose,									//specific target pose to match
			targetVel,									//specific target vel to match
			subGWtDivSds,								//precalced wts divided by sds
			subGWtDivSqSds,								//precalced wts divided by squared sds
			subGWts,									//weights for each of the component subgoals constituating a goal -> should be between 0 and 1
			subGMean,									//tar means for each subgoal
			subGSds;									//tar standard devs for each subgoal
		std::vector<std::string> subGNames;				//names of subgoal, for ui;
		std::vector<bool> subGFlags;					//enable/disable various subgoals - idx consts defined in each goal
		std::vector<bool> subGUseVar;					//whether or not this subgoal is used in variance caluclation
		int numSubGoals;								//# of subgoals
		double fullPoseSd,								//this sd value is on a per-bone/node basis, when matching an entire pose. it enables individual bone weighting based on how much we want a particular bone to be matched.  used for rest and target pose matching
			newLowVarLvlCost, newHiVarLvlCost,
			lowVarLvlCost = 10,							//set these by UI - these cap variance based on result, can vary widely, depend on result range->make min/max result values?
			highVarLvlCost = 400,
			maxLegDist,
			costMult,									//cost multiplier to scale costs							
			rHipHght,
			rCOMHght,
			rFootWdth;								//the rest pose foot separation
		//set restpose variables - fwd,up vectors, heightofcom, ground loc, etc.
		Eigen::Vector3d
			upVec,										//global up vector
			rp_Up,										//unit rest pose vectors for directions, should be orthonormal
			rp_Fwd,
			rp_Rght,
			rHeadRelCOM, 
			reachHandTarget,							//target to reach toward
			gnd_loc;									//location of a point on the ground plane
		int reachHandIDX;								//which hand is reaching

		static const int numUICmdBtns = 5;

		std::vector<bool> SM;							//various boolean values used by all goals
		static const int updEvryStp = 0;				//whether or not this goal is updated every step of sim
		static const int updVarLvlStp = 1;				//whether or not this goal should update var level bounds every step
		static const int useSqCosts = 2;				//whether to use squared costs or not
		static const int showUI = 3;					//whether UI should be hidden or not

		const int numSMFlags = 4;

		std::shared_ptr<MyGuiHandler> UI;				//UI to enable modification of sim variables used by this motion goal 

		vector<int> ubIdxs;								//idx's of upper body dofs
	};//MotionGoal
}

#endif  // APPS_PARTICLEBELIEFPROP_MotionGoal_H_
/**
//new map of rb and dof idxs for dart 5.0
rigid bodies

[0]	"h_pelvis"
[1]	"h_thigh_left"
[2]	"h_shin_left"
[3]	"h_heel_left"
[4]	"h_toe_left"
[5]	"h_thigh_right"
[6]	"h_shin_right"
[7]	"h_heel_right"
[8]	"h_toe_right"
[9]	"h_abdomen"
[10] "h_spine"
[11] "h_head"
[12] "h_scapula_left"
[13] "h_bicep_left"
[14] "h_forearm_left"
[15] "h_hand_left"
[16] "h_scapula_right"
[17] "h_bicep_right"
[18] "h_forearm_right"
[19] "h_hand_right"

dof names
[0]	"j_pelvis_rot_x"
[1]	"j_pelvis_rot_y"
[2]	"j_pelvis_rot_z"
[3]	"j_pelvis_pos_x"
[4]	"j_pelvis_pos_y"
[5]	"j_pelvis_pos_z"

[6]	"j_thigh_left_z"
[7]	"j_thigh_left_y"
[8]	"j_thigh_left_x"

[9]	"j_shin_left"

[10] "j_heel_left_1"
[11] "j_heel_left_2"
[12] "j_toe_left"

[13] "j_thigh_right_z"
[14] "j_thigh_right_y"
[15] "j_thigh_right_x"
[16] "j_shin_right"
[17] "j_heel_right_1"
[18] "j_heel_right_2"
[19] "j_toe_right"
[20] "j_abdomen_1"
[21] "j_abdomen_2"
[22] "j_spine"

[23] "j_head_1"
[24] "j_head_2"

[25] "j_scapula_left"
[26] "j_bicep_left_z"
[27] "j_bicep_left_y"
[28] "j_bicep_left_x"
[29] "j_forearm_left"
[30] "j_hand_left"

[31] "j_scapula_right"
[32] "j_bicep_right_z"
[33] "j_bicep_right_y"
[34] "j_bicep_right_x"
[35] "j_forearm_right"
[36] "j_hand_right"
*/
