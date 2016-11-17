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
#include "CntxtLauncher.h"
namespace cPBPropApp {

	CntxtLauncher::CntxtLauncher(std::vector<std::shared_ptr<SimContext>>& _cntxtList, int _numCntxts):cntxtList(_cntxtList), numCntxts(_numCntxts) {	}
	CntxtLauncher::~CntxtLauncher() {}

	void CntxtLauncher::launchContexts(ControlPBP& pbp) {
		std::vector<std::shared_ptr<SimContext>>::iterator it;
		for (it = cntxtList.begin(); it != cntxtList.end(); ++it) {
			(*it)->deriveControl(pbp);
		}	
	}//launchContexts
	//void CntxtLauncher::updateCntxtRes(std::vector<std::shared_ptr<MotionGoal>>& goals) {
	//	std::vector<std::shared_ptr<SimContext>>::iterator it;
	//	for (it = cntxtList.begin(); it != cntxtList.end(); ++it) {
	//		(*it)->updateStateControl(0);
	//		//double tmpCost = 0,	varLevel = 0, maxVarLvl = 0;
	//		//for (int gidx = 0; gidx < goals.size(); ++gidx) {
	//		//	tmpCost += goals[gidx]->calcFrmSqCost((*it), varLevel, (*it)->deltaT * (*it)->stepIDX);
	//		//	maxVarLvl = (varLevel > maxVarLvl ? varLevel : maxVarLvl);
	//		//}
	//		//(*it)->updateVarAndCostPerStep(tmpCost, maxVarLvl);
	//	}

	//}
}//namespace
