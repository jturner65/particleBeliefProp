/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
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

#include "MotionGoal.h"
#include "SimContext.h"
#include "CPBPHandler.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/DegreeOfFreedom.h"
#include "dart/dynamics/Joint.h"
#include "dart/dynamics/Marker.h"
#include "dart/dynamics/PointMass.h"
#include "dart/dynamics/SoftBodyNode.h"
using namespace std;


namespace cPBPropApp {
	void MotionGoal::init() {//initialize common subgoal constructs
	}

	void MotionGoal::refresh() {
	}

	void MotionGoal::setRestState(std::shared_ptr<SimContext> simCtxt) {
		//initialize goal with rest pose from source sim context
		cout << "MotionGoal::setRestState from cntxt : " << simCtxt->name << "\n";
		restState = simCtxt->restState;
		restPose = restState.head(simCtxt->numDofs);
		int idx = 0;
		for (std::vector<int>::iterator it = ubIdxs.begin(); it != ubIdxs.end(); ++it) {
			restPoseUpper[idx++] = restPose[*it];
		}
		rHeadRelCOM = simCtxt->nodes[simCtxt->cp->headRBIdx]->getCOM() - simCtxt->COM;							//vector from COM to Head COM
		restDofVel = restState.tail(simCtxt->numDofs);
		Eigen::Vector3d lFootCOM = simCtxt->nodes[simCtxt->cp->lftHeelRBIdx]->getCOM(),							
			rFootCOM = simCtxt->nodes[simCtxt->cp->rtHeelRBIdx]->getCOM(),
			lToeCOM = simCtxt->nodes[simCtxt->cp->lftToeRBIdx]->getCOM(),										//tmp vars for foot and toe com locations for rest state
			rToeCOM = simCtxt->nodes[simCtxt->cp->rtToeRBIdx]->getCOM();
		//set restpose variables - fwd,up vectors, heightofcom, ground loc, etc.
		double heelWidth = (lFootCOM - rFootCOM).norm(),
			toeWidth = (lToeCOM - rToeCOM).norm();
				
		rFootWdth = .5*(heelWidth + toeWidth);																	//rest post foot width - avg of heel width and toe width
		rp_Up<<0,1,0;
		Eigen::Vector3d rtFwdVec(0,0,0), ltFwdVec(0,0,0);
		rtFwdVec = (simCtxt->nodes[16]->getCOM() - simCtxt->nodes[9]->getCOM());		rtFwdVec.normalize();		//right shoulder - abdomen
		ltFwdVec = (simCtxt->nodes[12]->getCOM() - simCtxt->nodes[9]->getCOM());		ltFwdVec.normalize();		//left shoulder - abdomen
		setFwdVec(ltFwdVec.cross(rtFwdVec));      //forward orientation direction of rest pose - vector from rest state abd to left shldr cross abd to rt shldr  - should also use drag vector

		gnd_loc = simCtxt->COPCalcPt;			//point on ground used to calculate cop location

		rHipHght = simCtxt->nodes[simCtxt->cp->pelvisRBIdx]->getCOM()(1) - rFootCOM(1);				//set rest state hip height as difference in y between rest state pelvis com and rest state foot com
		//rest com height
		rCOMHght = simCtxt->COM(1) - rFootCOM(1);													//set rest state com height as difference in y between rest state com and rest state foot com

		//below only set to not NaN in comparison if erroneously enabled - TODO use to measure cost against specific non-rest pose
		targetState = simCtxt->restState;
		targetPose = targetState.head(simCtxt->numDofs);
		targetVel = targetState.tail(simCtxt->numDofs);
	}
	//reset fwd vector, recalc right vec
	void MotionGoal::setFwdVec(const Eigen::Vector3d &fwd) {
		rp_Fwd = fwd;      //forward orientation direction of rest pose - vector from rest state abd to left shldr cross abd to rt shldr  - use drag vector instead
		rp_Fwd.normalize();
		rp_Rght = rp_Fwd.cross(rp_Up);
	}
	void MotionGoal::setTargetState(std::shared_ptr<SimContext> simCtxt) {
		//use current pose of passed skeleton in simCtxt as target pose for this goal for all skeletons to consume it
		targetState << simCtxt->currState;
		targetPose << targetState.head(simCtxt->numDofs);
		targetVel << targetState.tail(simCtxt->numDofs);
	}
	//normalize weights, set pre-calced wt/sd and wt/(sd*sd) - if redist, reset all weights to be same value for every active subgoal
	void MotionGoal::updateSubGWtMultSds(bool reDist) {
		Eigen::VectorXd tmpWeights(numSubGoals);
		double wtSum = 0;
		//For subgoal obj : getWeight
		//for (int i = 0; i < numSubGoals; ++i) { tmpWeights[i] = subgoals[i].getWeight(); wtSum += tmpWeights[i]; }			//inactive subgoals goals have 0 weight
		//tmpWeights /= wtSum;
		for (int i = 0; i < numSubGoals; ++i) { tmpWeights[i] = (subGFlags[i] ? (reDist ? 1 : subGWts(i)) : 0); wtSum += tmpWeights[i]; }			//inactive subgoals goals have 0 weight
		subGWts = tmpWeights;
		subGWts /= wtSum;
		for (int i = 0; i < numSubGoals; ++i) {
			//call setwt here for each goal with tmpWeights[i]
			//subgoal[i].setWeight(tmpWeights[i]);
			subGWtDivSds[i] = subGWts(i) / subGSds(i);
			subGWtDivSqSds[i] = subGWts(i) / (subGSds(i) * subGSds(i));
		}
		cout <<"MotionGoal::updateSubGWtMultSds call Results : \n"<< *this << endl;
	}

	vector<double> MotionGoal::accumulateVals() {		//grab all locals that are modifiable by UI or used elsewhere for UI display
		vector<double> res, tmpRes;
		//get all current variable values from weights, means and sds to send to UI
		for (int idx = 0; idx < numSubGoals; ++idx) {
			//from subgoal obj : 
			//tmpRes = subgoals[idx].accumulateToUIVals();
			//res.insert(std::end(res), std::begin(tmpRes), std::end(tmpRes));
			res.push_back(subGWts(idx));
			res.push_back(subGMean(idx));
			res.push_back(1.0/subGSds(idx));
		}
		res.push_back(costMult);
		res.push_back(fullPoseSd);						//this sd value is on a per-bone/node basis, when matching an entire pose. it enables individual bone weighting based on how much we want a particular bone to be matched.  used for rest and target pose matching
		res.push_back(lowVarLvlCost);					//set these by UI - these cap variance based on result, can vary widely, depend on result range->make min/max result values?
		res.push_back(highVarLvlCost);
		return res;
	}

	void MotionGoal::distributeVals(vector<double>& vals) {
		int ridx = 0;
		for (int idx = 0; idx < numSubGoals; ++idx) {
			//sending to subgoal obj : 
			//subgoals[idx].setFromUIVals(vals[ridx],vals[ridx+1],vals[ridx+2]);		//undefined order of incrementing to increment index in argument list
			//ridx +=3;
			subGWts(idx) = vals[ridx++];
			subGMean(idx) = vals[ridx++];
			subGSds(idx) = 1.0 / vals[ridx++];
		}
		updateSubGWtMultSds(false);								//call whenever weights, mults, or sds change - normalizes weights among active subgoals - only pass true if # of subgoals has changed (increased)
		costMult = vals[ridx++];
		fullPoseSd = vals[ridx++];						//this sd value is on a per-bone/node basis, when matching an entire pose. it enables individual bone weighting based on how much we want a particular bone to be matched.  used for rest and target pose matching
		lowVarLvlCost = vals[ridx++];					//set these by UI - these cap variance based on result, can vary widely, depend on result range->make min/max result values?
		highVarLvlCost = vals[ridx++];
	}

	//go through the UI objects, find what has been clicked
	bool MotionGoal::handleBtnClick() {
		bool result = false;
		for (int i = 0; i < UI->MyPgUIBtn.size(); i++) {
			if (UI->MyPgUIBtn[i]->isClicked()) {
				std::string onclick = UI->MyPgUIBtn[i]->getOnClick();
				cout << "MotionGoal handleBtnClick() : " << name << " Button Clicked ID : " << i << "|" << UI->MyPgUIBtn[i]->getLabel() << " onclick = " << onclick << endl;
				switch (i) {
					case 0: {getValsFromUI(); return true; }//"Send Values to goal from UI",
					case 1: {resetUIWithDefVals(); return true; }//"Reset Values for ui and goal",
					case 2: {updateSubGWtMultSds(true); sendValsToUI();	return true; }//"reNormalize weights",
					case 3: {saveUIVals(); return true; }//"Save Values to file",
					case 4: {setSMBool(showUI, !SM[showUI]); return true; }//"Hide UI",
					default: {
						if ((i == numSubGoals + numUICmdBtns) || (i == numSubGoals + 1 + numUICmdBtns)){//turn on/off squared costs or variable variance levels
							SM[i - numUICmdBtns - numSubGoals + 1] = UI->MyPgUIBtn[i]->isChecked;
						}
						else if ((UI->MyPgUIBtn[i]->isCheckBox) && (i < numSubGoals + numUICmdBtns)) {//turn on/off subgoal from check box
							bool turnOnSG = UI->MyPgUIBtn[i]->isChecked;
							subGFlags[i - numUICmdBtns] = turnOnSG;
							updateSubGWtMultSds(false);			//turn off a subgoal preserves old weights relative(zeros disabled subgoal), turn on a subgoal requires it to be re-weighted
							sendValsToUI();
							return true;
						}			
						else { return false; }
					}		//if outside the range of the specified values then may be goal specific button
				}
			}//if click
		}//for each button
		return false;
	}

	//build components of UI universal to all goals
	void MotionGoal::buildUI() {
		//	//TODO : read this in from XML?
		//cout << "Motion Goal : " << name << " UI build start" << endl;
		string buttonNames[numUICmdBtns] = { string("Set Vals"), string("Reset Vals"), string("Redist Wts"), string("Save Vals"), string("Hide UI") };			//main named buttons
		const int numLstSldr = 4;		//final sliders after motion goals
		string lstSldrNames[numLstSldr] = { string("Cost Mult : "), string("Full Pose SD : "), string("Low Var Level : ") , string("High Var Level :")	};			//main named buttons
		string sldrNames[3] = { string("Weight : "), string("Target Val : "), string("Inv STD : ") };			//main named buttons

		//build subgoal-directly related components here, add components particular to each goal in the child class
		//int numSliderCaps = numSubGoals;					//# of related slider clusters (caption for subgoal name, caption for subgoal cost eval, bool for subgoal on/off, wt slider, mean slider, sd slider

		int numObjs = numUICmdBtns + (2 * numLstSldr) + 4 + numSubGoals * 8;		//total # of UI objects: numBtns + (2*numLastSliders) + 4 + numSubGoals * 8
		//int numObjs = numUICmdBtns + 6 + 4 + numSubGoals * 8;		//total # of UI objects: numBtns + (2*numLastSliders) + 4 + numSubGoals * 8

		UI->numObjs += numObjs;								//# of ui objects
		vector<int> objType;								//what type each object is : 0 : button, 1 : slider, 2 : caption, 3 : textbox
		vector<string> objLabels;							//object labels
		vector<vector<int>> objXY_WH;
		vector<vector<float>> objClr;

		objType.resize(numObjs);								//what type each object is : 0 : button, 1 : slider, 2 : caption, 3 : textbox
		objXY_WH.resize(numObjs);
		objLabels.resize(numObjs);
		objClr.resize(numObjs);
		int idx = 0;
		stringstream ss;

		// wt slider (0-1), mult slider(1->100), sd slider(.0001 ->10), mean slider(-100->100)
		float sldrMinVals[3] = { 0.0001f, -10, .01f };				//all sliders have same min and max bounds : wt slider,mean slider, sd slider
		float sldrMaxVals[3] = { 0.9999f, 10, 1000 };					//all sliders have same min and max bounds
		float smallSldrLen = 90;
		vector<float> dims;
		dims.resize(14);

		dims[0] = 18;								//sliderLen = 18
		dims[1] = 13;								//sliderWide = 13
		dims[2] = 46;								//widBuf;
		dims[3] = 30;								//capWide = 30
		dims[4] = smallSldrLen + dims[2];			//col2 = smallSldrLen + widBuf = 90 + 46
		dims[5] = .5f*dims[2];						//hWidBuf;
		dims[6] = .25f*dims[2];						//qWidBuf;
		dims[7] = 14;								//_stOff = 14;
		dims[8] = 7;								//capBuf = 7;
		dims[9] = 28;								//yBuffer2 = sliderLen + 10 = 28;
		dims[10] = dims[9] + dims[5];							//capYBuff = yBuffer2 + hWidBuf;
		dims[11] = 10;								//initY = 10
		dims[12] = 25;								//btnHigh = 25
		dims[13] = (dims[2] * (numSubGoals + 2)) + dims[6]; // stY = (widBuf*(numSubGoals + 2)) + qWidBuf;

		UI->height = dims[13] + dims[6] + dims[12];

		UI->width = UI->winX - UI->stX - 1;			//extends to edge of screen
		int numLstSlPRow = 2;
		float perBtnWidth = (numUICmdBtns > 0 ? UI->width / numUICmdBtns : 0), btnWide = .85* perBtnWidth, btnBuff = (numUICmdBtns > 0 ? (UI->width - (btnWide*numUICmdBtns)) / (numUICmdBtns - 1) : 0),
			perLstSldrWid = (numLstSlPRow > 0 ? (UI->width) / numLstSlPRow : 0);// , lstSldrWid = sliderWide;


		//ui comps
		//interaction buttons
		for (int i = 0; i < numUICmdBtns; ++i) {
			ss.str("");		ss << buttonNames[i];
			vector<float> v1 = { (i*perBtnWidth), dims[13] + dims[6], btnWide, dims[12], .75f, .75f, .75f, 1 };
			UI->initUIObj(idx++, ss.str(), 0, v1, objType, objLabels, objXY_WH, objClr);
		}

		//for each subgoal
		for (int i = 0; i < numSubGoals; i ++) {		//1 column, each with button, caption, and  3 sliders
			float widBufMult = dims[2] * i;
			ss.str("");		ss <<"Enable";
			vector<float> vb = { 0, widBufMult, dims[0], dims[0], .75f, .75f, .75f, 1 };
			UI->initUIObj(idx++, ss.str(), 0, vb, objType, objLabels, objXY_WH, objClr);
			ss.str("");		ss << subGNames[i] << " cost : ";
			vector<float> vCap = { dims[5], dims[7] + widBufMult, dims[3], dims[3], .25f, .25f, .25f, 1 };
			UI->initUIObj(idx++, ss.str(), 2, vCap, objType, objLabels, objXY_WH, objClr);

			for (int j = 0; j < 3; ++j) {//3 sliders weight/tar val/mult
				float multX = j * dims[4];
				vector<float> v1 = { (multX)+dims[1], (widBufMult + dims[0]), dims[1], dims[0], .85f, .85f, .85f, 1 };		//sliderwide is width of thumb
				UI->initUIObj(idx++, "", 1, v1, objType, objLabels, objXY_WH, objClr);		//puts appropriate values in array
				ss.str("");		ss << sldrNames[j];
				vector<float> v2 = { (multX)+dims[1] + dims[1], (widBufMult + dims[10]) - dims[8], dims[3], dims[3], .25f, .25f, .25f, 1 };
				UI->initUIObj(idx++, ss.str(), 2, v2, objType, objLabels, objXY_WH, objClr);
			}
		}
		//buttons after subgoals
		for (int j = 0; j < 2; ++j) {
			float multY = (dims[2] * (numSubGoals + (.75*j)) + dims[9]) - 2 * dims[11], multCapY = multY + 4 * dims[8];
			for (int i = 0; i < numLstSldr / 2; ++i) {
				float multX = i* perLstSldrWid;
				vector<float> v1 = { (multX)+dims[1], multY, dims[1], dims[0], .85f, .85f, .85f, 1 };
				UI->initUIObj(idx++, "", 1, v1, objType, objLabels, objXY_WH, objClr);		//puts appropriate values in array
				ss.str("");		ss << lstSldrNames[(j*2) + i];
				vector<float> v2 = { (multX)+dims[1] + dims[1], multCapY, dims[3], dims[3], .25f, .25f, .25f, 1 };
				UI->initUIObj(idx++, ss.str(), 2, v2, objType, objLabels, objXY_WH, objClr);
			}
		}

		string cbNames[2] = { "Use Variable Var Lvls", "Use Squared Costs" };
		for (int i = 0; i < 2; ++i) {
			//variable variance update - every cycle modify low and hi var levels to be equal to min and max costs of last time and squared costs enable/disable
			ss.str("");		ss << "Enable";
			vector<float> vb = { (i*(dims[4] + dims[1] + dims[1])), dims[13] - dims[0], dims[0], dims[0], .75f, .75f, .75f, 1 };
			UI->initUIObj(idx++, ss.str(), 0, vb, objType, objLabels, objXY_WH, objClr);
			ss.str("");		ss << cbNames[i];
			vector<float> vCap = { (i*(dims[4] + dims[1] + dims[1])) + dims[5], dims[7] + dims[13] - dims[0], dims[3], dims[3], .25f, .25f, .25f, 1 };
			UI->initUIObj(idx++, ss.str(), 2, vCap, objType, objLabels, objXY_WH, objClr);
		}

			//send to UI handler to render objs
		UI->setUI(objType, objLabels, objXY_WH, objClr, smallSldrLen);
		//set caption for each slider
		UI->MyPgUISldr[0]->setCaption(UI->MyPgUICaption[0], UI->MyPgUISldr[0]);
		for (int i = 0; i < numSubGoals; ++i) {
			for (int j = 0; j < 3; ++j) {
				int SldrIdx = (i * 3) + j , capIdx = (4 * i) + j + 1 ;
				UI->MyPgUISldr[SldrIdx]->setCaption(UI->MyPgUICaption[capIdx], UI->MyPgUISldr[SldrIdx]);
			}
		}
		for (int j = 0; j < numLstSldr; ++j) {
			int SldrIdx = (numSubGoals * 3) + j, capIdx = (4 * numSubGoals) + j;
			UI->MyPgUISldr[SldrIdx]->setSbDimW(perLstSldrWid * .80f);
			UI->MyPgUISldr[SldrIdx]->setCaption(UI->MyPgUICaption[capIdx], UI->MyPgUISldr[SldrIdx]);
		}

		for (int i = 0; i < UI->MyPgUISldr.size() - numLstSldr; i += 3) {
			for (int j = 0; j < 3; ++j) {
				UI->MyPgUISldr[i+j]->setSlideMin(sldrMinVals[j]);
				UI->MyPgUISldr[i+j]->setSlideRng(sldrMaxVals[j] - sldrMinVals[j]);
			}
		}
		int lastSldr = UI->MyPgUISldr.size() - numLstSldr;
		float sldrLastMinVals[numLstSldr] = { 0.0001f, 0.0001f, 0.0001f, 10 };				//for last 3 sliders- full pose sd, lowvarlevel and high var level
		float sldrLastMaxVals[numLstSldr] = { 1, 10, 100, 500 };

		for (int j = 0; j < numLstSldr; ++j) {
			UI->MyPgUISldr[lastSldr + j]->setSlideMin(sldrLastMinVals[j]);
			UI->MyPgUISldr[lastSldr + j]->setSlideRng(sldrLastMaxVals[j] - sldrLastMinVals[j]);
		}
		//set UI show/hide button
		UI->setShowHideBtn(numUICmdBtns - 1);//last cmd btn is hide btn

		for (int i = 0; i < UI->MyPgUIBtn.size(); ++i) { 
			if (i < numUICmdBtns) { UI->MyPgUIBtn[i]->flags[UI->MyPgUIBtn[i]->UIobjIDX_CtrLbl] = true; }
			else { 
				UI->MyPgUIBtn[i]->flags[UI->MyPgUIBtn[i]->UIobjIDX_CtrLbl] = false;  UI->MyPgUIBtn[i]->isCheckBox = true;
				if (i - numUICmdBtns < numSubGoals) { UI->MyPgUIBtn[i]->isChecked = subGFlags[i - numUICmdBtns]; }		//turn on/off subgoals
				else { 
					UI->MyPgUIBtn[i]->isChecked = SM[i - numUICmdBtns - numSubGoals + 1];
				}//toggle between squared and linear cost calc or using adaptive var levels
			}
		}//center button captions on first 3 buttons, set rest to be checkboxes
	//	cout << "Motion Goal : "<<name<<" UI build end :\n" <<*this<< endl;
	}

	//get values and send to UI
	void MotionGoal::sendValsToUI() {
		vector<double> res = accumulateVals();
		for (int i = 0; i < UI->MyPgUISldr.size(); ++i) { UI->MyPgUISldr[i]->setSliderVal((float)(res[i])); }
		cout << "\nMotionGoal::sendValsToUI() Send Internal Vals To UI : " << *this;
	}

	void MotionGoal::resetValues(bool dummy) {//set some reasonable defaults in here that we can return to
		init();
		UI->MyPgUIBtn[numSubGoals + 3]->isChecked = SM[updVarLvlStp];		//update check box for whether squared costs are used or not
		UI->MyPgUIBtn[numSubGoals + 3 + 1]->isChecked = SM[useSqCosts];		//update check box for whether squared costs are used or not
	}

	void MotionGoal::getValsFromUI() {
		//get all slider vals
		vector<double> vals;
		for (int i = 0; i < UI->MyPgUISldr.size(); ++i) { vals.push_back(UI->MyPgUISldr[i]->getCurValue()); }
		distributeVals(vals);			//set local values from UI
		cout << "\nMotionGoal::getValsFromUI() Get UI data and set internal vals : " << *this;
		sendValsToUI();					//refresh UI with values - verify that all values are being set properly
	}

	void MotionGoal::resetUIWithDefVals() {		resetValues(true);		sendValsToUI();	}
	void MotionGoal::saveUIVals() {	cout << "Not implemented yet" << endl;}

	//set boolean values that drive goal evaluation, handling anything special that may be necessary for 
	//particular flags for state machine (not subgoals)
	void MotionGoal::setSMBool(int idx, bool val) {
		SM[idx] = val;
		switch (idx) {
			case updEvryStp: {break; }
			case updVarLvlStp: {break; }
			case useSqCosts: {break; }
			case showUI: {
				UI->setShowUI(val);				
				break; }//turn on/off ui by calling function in  UI handler   useSqCosts
		}
	}

	std::ostream& operator<<(std::ostream& out, MotionGoal& mg) {
		out << "Goal Name : " << mg.name << "\t# Of Subgoals : " << mg.numSubGoals 
			<< "\tUpdate Every Physics step? : " << (mg.SM[mg.updEvryStp] ? "True " : "False") 
			<< "\tUpdate Var Level Bounds step? : " << (mg.SM[mg.updVarLvlStp] ? "True " : "False")
			<< "\tUse Squared Costs? : " << (mg.SM[mg.useSqCosts] ? "True " : "False")
			<< "\n";
		out << "Full Pose SD : " << mg.fullPoseSd << "\tLow Var lvl cost : " << mg.lowVarLvlCost << "\tHign Var lvl cost : " << mg.highVarLvlCost 
			<< "\tRest Hip Height : " << mg.rHipHght << "\tRest Com Height : " << mg.rCOMHght << "\tRest Foot Width : " << mg.rFootWdth << "\n";
		out << "Global Up : (" << mg.upVec[0] << "," << mg.upVec[1] << "," << mg.upVec[2]<< ")\tPose Up : (" << mg.rp_Up[0] << "," << mg.rp_Up[1] << "," << mg.rp_Up[2]
			<< ")\tPose Fwd : (" << mg.rp_Fwd[0] << "," << mg.rp_Fwd[1] << "," << mg.rp_Fwd[2]<< ")\tPose Right : (" << mg.rp_Rght[0] << "," << mg.rp_Rght[1] << "," << mg.rp_Rght[2] << ")\n";
		out << "Rest Head Loc : (" << mg.rHeadRelCOM[0] << "," << mg.rHeadRelCOM[1] << "," << mg.rHeadRelCOM[2] << ")\tGlobal Ground Loc : (" << mg.gnd_loc[0] << "," << mg.gnd_loc[1] << "," << mg.gnd_loc[2] << ")\n";
		out << "Sub goals :\n";
		for (int i = 0; i < mg.numSubGoals; ++i) {
			out << "\tIdx : " << (i < 10 ? " ":"")<<i << "\tEnabled? : " << (mg.subGFlags[i] ? "True " : "False") << "\tWeight : " << mg.subGWts(i) << "\tMean : " << mg.subGMean(i) << "\tStd : " << mg.subGSds(i) << "\tName : " << mg.subGNames[i] << "\n";
		}
		return out;
	}
}