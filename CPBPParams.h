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

#ifndef APPS_PARTICLEBELIEFPROP_CPBPParams_H_
#define APPS_PARTICLEBELIEFPROP_CPBPParams_H_

#include <vector>
#include <string>

#include <iostream>

#include <Eigen/Dense>
#include <tinyxml2.h>
#include "dart/utils/Parser.h"
#include <boost/algorithm/string.hpp>

namespace cPBPropApp {

	//this class handles the basic hyperparams for c-pbp.  an instance is held in a shared pointer whose primary owner is the CPBPHandler instance, and is shared with all sim cntxts
	class CPBPParams {//only 1 copy of this should be made - eventually make a singelton?
	public:

		CPBPParams();
		virtual ~CPBPParams() {}
		//bool lclGetValueBool(tinyxml2::XMLElement* _parentElement, const std::string& _name);

		void setStateRBI(const Eigen::Ref<const Eigen::VectorXd>& bodyIdxs) {
			numStateBodies = bodyIdxs.size();
			stateRBIdxs.resize(numStateBodies);
			for (int i = 0; i < stateRBIdxs.size(); ++i) { stateRBIdxs[i] = bodyIdxs(i); }
		}
		void setPlanHorizAndFwdTS(double _ph, double _delT);

		std::vector<double> accumulateVals();
		void distributeVals(std::vector<double>& vals);
		void setDefaultVals();
		void setCurrentValsAsDefault();
		void setParamValFromXMLStr(const std::string& _name, const std::string& s);// , tinyxml2::XMLElement* _parentElement);
		//void setParamValFromXML(const std::string& _name, double _vald, int _valI, bool valb, std::string& _valstr);

		void resetValues();

		friend std::ostream& operator<<(std::ostream& out, CPBPParams& cpbp);

	public:
		std::vector<double> defaultVals;
		std::vector<double>lowEffortSds, highEffortSds, lowEffortFMax, highEffortFMax;
		std::vector<int> stateRBIdxs, upperBodyRBIdxs;
		std::string skelFileName, skelType;													//FULL path to skel (including DART data path, if used; skeleton type	

		const int pelvisRBIdx = 0, lftHeelRBIdx = 3, rtHeelRBIdx = 7, headRBIdx = 11, lftToeRBIdx = 4, rtToeRBIdx = 8, lftHandRBIdx = 15, rtHandRBIdx = 19;	//skeleton params

		double resampleThreshold, uniformBias, posePriorStd, stateKernelScale, gbpRegularization,
			jerkSd,	baseStdScale, footStdScale, kneeStdScale, armStdScale,
			mutationScale, deltaT, planHorizonSec, cntrlPrSDMult,
			fMaxSDMult, invDelTNumStpsPerThd, maxCntlVal,
			motorFmax, torsoMinFMax, legsMinFMax, armsMinFMax;								//min force exerted for motor values 

		int	numFwdTimeSteps,																//derived by time step, # of time step sims per control derivation and length of forward prediction step
			idxInWorld,												//skeleton index in world
			numSamples,												//N in the paper
			numCntrlFlim,											//control has extra 5 values of force limits corresponding to bone category types (torso, arms, legs)
			numStateBodies,											//# of rigid bodies used in reduced state
			numSimFwdStepsPerThd;									//# of forward sim steps to take in each thread per control derivation

		std::vector<bool> flags;									//various boolean flags used to drive cpbp
		static const int IDX_useGBP		= 0;						//whether or not to use gauss back prop
		static const int IDX_useStKnStd = 1;						//whether or not to use state kernel std
		static const int IDX_useVelMtr = 2;							//whether or not to use velocity based motors (as opposed to torque-based)
		static const int IDX_useJerkCost = 3;						//whether or not to use jerk cost in jerk calculation
		static const int IDX_useKLReg = 4;							//TODO : whether or not to use kernelized log regression (gpr with loss functional grad descent) for back-smoothing instead of gbp

		static const int numFlags = 5;
	};

}//namespace cPBPropApp 
#endif  // APPS_PARTICLEBELIEFPROP_CPBPParams_H_
