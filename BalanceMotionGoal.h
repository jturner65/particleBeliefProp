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

#ifndef APPS_PARTICLEBELIEFPROP_BalanceMotionGoal_H_
#define APPS_PARTICLEBELIEFPROP_BalanceMotionGoal_H_

#include <vector>
#include <string>
#include <memory>
#include <Eigen/Dense>
#include "MotionGoal.h"
namespace cPBPropApp {
	//the goal is to stay balanced
	class BalanceMotionGoal : public MotionGoal {
	public:
		BalanceMotionGoal(int _numSubG, const vector<int>& _ubRbIdx);
		virtual ~BalanceMotionGoal() {}

		virtual void init();
		virtual void refresh();
		virtual double calcFrmCost(std::shared_ptr<SimContext> simCtxt, double& varLevel, double timeSec);
		virtual void setRestState(std::shared_ptr<SimContext> simCtxt);
		virtual void setTargetState(std::shared_ptr<SimContext> simCtxt);
		//build the UI construct to modify the values of this class
		//virtual void initUIObj(int idx, string label, int type, vector<float>& vals, vector<int>& objType, vector<string>& objLabels, vector<vector<int>>& objXY_WH, vector<vector<float>>& objClr);
		virtual void buildUI();
		virtual void sendValsToUI();
		virtual void initUI() { buildUI();	sendValsToUI(); }	//virtual void buildUI();

		virtual void resetValues(bool sendToCntxts);
		virtual void getValsFromUI();
		virtual vector<double> accumulateVals();
		virtual void distributeVals(vector<double>& vals);
		virtual void resetUIWithDefVals();
		virtual void saveUIVals();
		//event handler for UI components related to this class
		virtual bool handleBtnClick();  //only for goal-specific buttons

		virtual void setSubGFlag(int idx, bool val, bool useResInVar);

		friend std::ostream& operator<<(std::ostream& out, BalanceMotionGoal& goal);

	public: //variables
		//sub goal boolean flags/weights/names
		static const int
			idxCOMCOP = 0,					//com-cop distance projected on a plane
			idxCOMDisp = 1,					//COM displacement from line between balls of feet
			idxCOMFwdVel = 2,				//fwd COM vel 
			idxCOMLatVel = 3,				//lat COM vel
			idxHeadRelCOM = 4,				//penalize head deviating position relative to COM
			idxFootDist = 5,				//foot separation that deviates from a certain value
			idxFootMaxDist = 6,				//penalize splits and giant strides - anything beyond a max distance
			idxFeetNoCross = 7,				//no leg crossing

			idxUp = 8,						//hipOrientation
			idxHipY = 9,					//hip height
			idxHipAng = 10,					//hip angular velocity(twisting)
			idxReaching = 11,				//reach toward target
			idxRestPose = 12,				//closeness to joint angles of the rest pose
			idxRestUpperBody = 13;			//closeness to joint angles of the upper body of the rest pose

			//idxFootHeight = 13,				//
			//idxTarPose = 14,				//closeness to joint angles of some target pose
			//idxTarUpperBody = 15,			//closeness to joint angles of the upper body of the target pose
			//idxTarVel = 16;

	};//BalanceMotionGoal
}//namespace cPBPropApp 
#endif  // APPS_PARTICLEBELIEFPROP_BalanceMotionGoal_H_
