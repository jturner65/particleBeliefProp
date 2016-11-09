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

#include "CPBPParams.h"

using namespace std;
using namespace Eigen;

namespace cPBPropApp {
	//reasonable initial values if no values loaded from xml
	//int CPBPParams::numSamples = 121;					//N in the paper - # of predictive walkers
	//int CPBPParams::numStateBodies = 6;				//# of rigid bodies used in reduced state - consists of com and comvel of each body, + root up orientation
	//int CPBPParams::numCntrlFlim = 5;				//control has extra 5 values of force limits corresponding to bone category types (torso, arms, legs)

	//int CPBPParams::numSimStepPerSample = 1;			//# of forward sim steps per simcontext per control derivation from PBP
	//double CPBPParams::stateKernelScale = 0;			// .01 * M_PI;		//stateKernelScale == 0 denotes not using state kernel

	CPBPParams::CPBPParams() :
		defaultVals(), lowEffortSds(4), highEffortSds(4), lowEffortFMax(4), highEffortFMax(4), stateRBIdxs(6), upperBodyRBIdxs(),
		skelFileName(""), skelType(""), resampleThreshold(.5), uniformBias(.25), posePriorStd(0.2f), stateKernelScale(0), 
		gbpRegularization(.001), jerkSd(.15), baseStdScale(1.0), footStdScale(0.5), kneeStdScale(1.0), armStdScale(0.75),
		mutationScale(.25), deltaT(1.0 / 60.0), planHorizonSec(.5), cntrlPrSDMult(1), fMaxSDMult(.08), //fMaxSDMult is magic number from Unity code that multiples the sd for the per-step priors for the force limits- UI enterable? hardcoded to 4.0 in unity code
		invDelTNumStpsPerThd(0), maxCntlVal(4.188790204786391), motorFmax(100), torsoMinFMax(20),legsMinFMax(20), armsMinFMax(1),
		numFwdTimeSteps(0), idxInWorld(1), numSamples(36), numCntrlFlim(5), numStateBodies(6), numSimFwdStepsPerThd(1), flags(numFlags, false)
	{
		cout << "CPBP params ctor\n";
		invDelTNumStpsPerThd = 1.0 / (deltaT*numSimFwdStepsPerThd);						//precalced per-fwdPrediction iteration constant - multiplied on pose prior mean
		for (int i = 0; i < numFlags; ++i) {flags[i] = false;}
		
		flags[IDX_useGBP] = true;
		flags[IDX_useVelMtr] = true;
		flags[IDX_useJerkCost] = false;												//as per communications with perttu 6/12/15
		setDefaultVals();																//set defaults from hardcoded values - overridden by xml data if necessary
		stateRBIdxs[0] = 0;																//pelvis
		stateRBIdxs[1] = 11;																//head
		stateRBIdxs[2] = 3;																//left heel
		stateRBIdxs[3] = 7;																//right heel
		stateRBIdxs[4] = 15;																//left hand
		stateRBIdxs[5] = 19;																//right hand
		int tmpUbiIdxs[13] = { 20, 21, 22, 23, 24, 25, 26, 27, 28, 31, 32, 33, 34 };			//dof idx's of upper body values - don't care about elbow and wrist dofs : 29, 30, 35, 36 
		upperBodyRBIdxs.insert(upperBodyRBIdxs.begin(), &tmpUbiIdxs[0], &tmpUbiIdxs[13]);	//need to build these from skeleton being read in
		setCurrentValsAsDefault();
	}


	void CPBPParams::setParamValFromXMLStr(const std::string& _name, const std::string& s) {
		if (_name.compare("numSamples") == 0) { numSamples = stoi(s);			return; }
		if (_name.compare("numStateBodies") == 0) { numStateBodies = stoi(s);        return; }
		if (_name.compare("numCntrlFlim") == 0) { numCntrlFlim = stoi(s);        return; }
		if (_name.compare("numSimFwdStepsPerThd") == 0) { numSimFwdStepsPerThd = stoi(s);        return; }
		if (_name.compare("idxInWorld") == 0) { idxInWorld = stoi(s);        return; }

		if (_name.compare("skelFileName") == 0) { skelFileName = std::string(s);	return; }
		if (_name.compare("skelType") == 0) { skelType = std::string(s);	return; }

		if (_name.compare("planHorizonSec") == 0) { planHorizonSec = stod(s);      return; }
		if (_name.compare("maxCntlVal") == 0) { maxCntlVal = stod(s);        return; }
		if (_name.compare("lowPosePriorStd") == 0) { lowEffortSds[0] = stod(s);        return; }
		if (_name.compare("lowControlPriorStd") == 0) { lowEffortSds[1] = stod(s);        return; }
		if (_name.compare("lowControlDiffPriorStd") == 0) { lowEffortSds[2] = stod(s);        return; }
		if (_name.compare("lowControlDiffDiffPriorStd") == 0) { lowEffortSds[3] = stod(s);        return; }

		if (_name.compare("highPosePriorStd") == 0) { highEffortSds[0] = stod(s);        return; }
		if (_name.compare("highControlPriorStd") == 0) { highEffortSds[1] = stod(s);        return; }
		if (_name.compare("highControlDiffPriorStd") == 0) { highEffortSds[2] = stod(s);        return; }
		if (_name.compare("highControlDiffDiffPriorStd") == 0) { highEffortSds[3] = stod(s);        return; }

		if (_name.compare("lowFMaxControlPriorStd") == 0) { lowEffortFMax[0] = stod(s);        return; }
		if (_name.compare("lowFMaxControlDiffPriorStd") == 0) { lowEffortFMax[1] = stod(s);        return; }
		if (_name.compare("lowFMaxControlDiffDiffPriorStd") == 0) { lowEffortFMax[2] = stod(s);        return; }

		if (_name.compare("highFMaxControlPriorStd") == 0) { highEffortFMax[0] = stod(s);        return; }
		if (_name.compare("highFMaxControlDiffPriorStd") == 0) { highEffortFMax[1] = stod(s);        return; }
		if (_name.compare("highFMaxControlDiffDiffPriorStd") == 0) { highEffortFMax[2] = stod(s);        return; }

		if (_name.compare("deltaT") == 0) { deltaT = stod(s);  invDelTNumStpsPerThd = 1.0 / (deltaT*numSimFwdStepsPerThd);      return; }
		if (_name.compare("resampleThreshold") == 0) { resampleThreshold = stod(s);        return; }
		if (_name.compare("uniformBias") == 0) { uniformBias = stod(s);        return; }
		if (_name.compare("gbpRegularization") == 0) { gbpRegularization = stod(s);        return; }
		if (_name.compare("baseStdScale") == 0) { baseStdScale = stod(s);        return; }
		if (_name.compare("footStdScale") == 0) { footStdScale = stod(s);        return; }
		if (_name.compare("kneeStdScale") == 0) { kneeStdScale = stod(s);        return; }
		if (_name.compare("armStdScale") == 0) { armStdScale = stod(s);        return; }
		if (_name.compare("jerkSd") == 0) { jerkSd = stod(s);        return; }
		if (_name.compare("stateKernelScale") == 0) { stateKernelScale = stod(s);        return; }
		if (_name.compare("motorFmax") == 0) { motorFmax = stod(s);        return; }
		if (_name.compare("mutationScale") == 0) { mutationScale = stod(s);        return; }
		if (_name.compare("torsoMinFMax") == 0) { torsoMinFMax = stod(s);        return; }
		if (_name.compare("legsMinFMax") == 0) { legsMinFMax = stod(s);        return; }
		if (_name.compare("armsMinFMax") == 0) { armsMinFMax = stod(s);        return; }

		if (_name.compare("useGBP") == 0) { flags[IDX_useGBP] = (s.compare("TRUE") == 0 ? true : false);        return; }
		if (_name.compare("useVelMtr") == 0) { flags[IDX_useVelMtr] = (s.compare("TRUE") == 0 ? true : false);    return; }
		if (_name.compare("useJerkCost") == 0) { flags[IDX_useJerkCost] = (s.compare("TRUE") == 0 ? true : false);     return; }
	}

	//set up reasonable default values to be used if XML is unavailable.  the default values will be used whenever reset is called
	void CPBPParams::setDefaultVals() {
		int lIdx = 0, hIdx = 0, lfIdx = 0, hfIdx = 0;
		posePriorStd = 0.2f;				//pose prior is modified by recompSampleParams based on effort		        
		fMaxSDMult = 4;						//fMaxSDMult is magic number from Unity code that multiples the sd for the per-step priors for the force limits- UI enterable? hardcoded to 4.0 in unity code

		stateKernelScale = 0;				//this can't be changed without re-initing ControlPBP		
		planHorizonSec = .5;				//this can't be changed without re-initing ControlPBP
		deltaT = 1.0 / 60.0;				//this can't be changed without re-initing ControlPBP
		numSimFwdStepsPerThd = 1;			//this can't be changed without re-initing ControlPBP
		invDelTNumStpsPerThd = 1.0 / (deltaT*numSimFwdStepsPerThd);		

		maxCntlVal = 4.188790204786391;     
		cntrlPrSDMult = 1;					//this can't be changed without re-initing CPBPhandler - used with min/max angles	 

		setPlanHorizAndFwdTS(planHorizonSec, deltaT);
		//UI enterable/changeable below
		lowEffortSds[lIdx++] = .1;				highEffortSds[hIdx++] = 1;
		lowEffortSds[lIdx++] = .5;				highEffortSds[hIdx++] = 2.5;
		lowEffortSds[lIdx++] = .1;				highEffortSds[hIdx++] = 1.0;
		lowEffortSds[lIdx++] = 1.0;				highEffortSds[hIdx++] = 10.0;

		lowEffortFMax[lfIdx++] = .1;			highEffortFMax[hfIdx++] = 1;
		lowEffortFMax[lfIdx++] = 0.1;			highEffortFMax[hfIdx++] = 0.5;
		lowEffortFMax[lfIdx++] = 1.0;			highEffortFMax[hfIdx++] = 10.0;//10 

		resampleThreshold = .5;					jerkSd = .15;
		uniformBias = .25;						motorFmax = 100;
		gbpRegularization = .001;				mutationScale = .25;
		baseStdScale = 1;						torsoMinFMax = 1;
		footStdScale = .5;						legsMinFMax = 1;
		kneeStdScale = 1.0;						armsMinFMax = 1;
		armStdScale = .75;						//add slider for fMaxSDMult
		flags[IDX_useGBP] = true;
		flags[IDX_useVelMtr] = true;
		defaultVals = accumulateVals();			//set default values as current param vals
	}//setDefaultVals

	void CPBPParams::setCurrentValsAsDefault() {
		defaultVals = accumulateVals();
	}//setCurrentValsAsDefault
	
	void  CPBPParams::setPlanHorizAndFwdTS(double _ph, double _delT) {
		planHorizonSec = _ph;
		deltaT = _delT;
		numFwdTimeSteps = (int)(planHorizonSec / (numSimFwdStepsPerThd * _delT));
		invDelTNumStpsPerThd = 1.0 / (deltaT*numSimFwdStepsPerThd);						//precalced per-fwdPrediction iteration constant - multiplied on pose prior mean
	}

	vector<double> CPBPParams::accumulateVals() {		//grab all locals that are modifiable by UI or used elsewhere
		vector<double> res;
		int idx = 0, lIdx = 0, hIdx = 0, lfIdx = 0, hfIdx = 0;
		res.push_back(planHorizonSec);				res.push_back(deltaT);

		res.push_back(lowEffortSds[lIdx++]);		res.push_back(highEffortSds[hIdx++]);
		res.push_back(lowEffortSds[lIdx++]);		res.push_back(highEffortSds[hIdx++]);
		res.push_back(lowEffortSds[lIdx++]);		res.push_back(highEffortSds[hIdx++]);
		res.push_back(lowEffortSds[lIdx++]);		res.push_back(highEffortSds[hIdx++]);

		res.push_back(lowEffortFMax[lfIdx++]);		res.push_back(highEffortFMax[hfIdx++]);
		res.push_back(lowEffortFMax[lfIdx++]);		res.push_back(highEffortFMax[hfIdx++]);
		res.push_back(lowEffortFMax[lfIdx++]);		res.push_back(highEffortFMax[hfIdx++]);

		res.push_back(resampleThreshold);			res.push_back(jerkSd);
		res.push_back(uniformBias);					res.push_back(motorFmax);
		res.push_back(gbpRegularization);			res.push_back(mutationScale);
		res.push_back(baseStdScale);				res.push_back(torsoMinFMax);
		res.push_back(footStdScale);				res.push_back(legsMinFMax);
		res.push_back(kneeStdScale);				res.push_back(armsMinFMax);
		res.push_back(armStdScale);					//add slider for fMaxSDMult
		return res;
	}

	void CPBPParams::distributeVals(vector<double>& vals) {//copy all UI values to their appropriate lcl variables (from UI or file)
		int idx = 0, lIdx = 0, hIdx = 0, lfIdx = 0, hfIdx = 0;
		setPlanHorizAndFwdTS(defaultVals[0], defaultVals[1]);// - dont' allow UI to change these values for now - require reiniting of cpbp setPlanHorizAndFwdTS(vals[idx++], vals[idx++]);//can't change this value without re-init of CPBP algorithm
		idx += 2;
		lowEffortSds[lIdx++] = vals[idx++];		highEffortSds[hIdx++] = vals[idx++];
		lowEffortSds[lIdx++] = vals[idx++];		highEffortSds[hIdx++] = vals[idx++];
		lowEffortSds[lIdx++] = vals[idx++];		highEffortSds[hIdx++] = vals[idx++];
		lowEffortSds[lIdx++] = vals[idx++];		highEffortSds[hIdx++] = vals[idx++];

		lowEffortFMax[lfIdx++] = vals[idx++];	highEffortFMax[hfIdx++] = vals[idx++];
		lowEffortFMax[lfIdx++] = vals[idx++];	highEffortFMax[hfIdx++] = vals[idx++];
		lowEffortFMax[lfIdx++] = vals[idx++];	highEffortFMax[hfIdx++] = vals[idx++];

		resampleThreshold = vals[idx++];		jerkSd = vals[idx++];
		uniformBias = vals[idx++];				motorFmax = vals[idx++];
		gbpRegularization = vals[idx++];		mutationScale = vals[idx++];
		baseStdScale = vals[idx++];				torsoMinFMax = vals[idx++];
		footStdScale = vals[idx++];				legsMinFMax = vals[idx++];
		kneeStdScale = vals[idx++];				armsMinFMax = vals[idx++];
		armStdScale = vals[idx++];				//add slider for fMaxSDMult
	}
	void CPBPParams::resetValues() {//set some reasonable defaults in here that we can return to them if not loaded from file
		distributeVals(defaultVals);
		flags[IDX_useGBP] = true;
	}

	std::ostream& operator<<(std::ostream& out, CPBPParams& p) {//for dbug output 
		out << "CPBP Params values : \n";
		out << "Skel Filename : " << p.skelFileName << " Skel Type : " << p.skelType << " Skel IDX in world : " << p.idxInWorld << "\t# Samples : " << p.numSamples << "\t# Force Limit Control dofs : " << p.numCntrlFlim<<"\n";
		out << "# Fwd Prediction Steps : " << p.numFwdTimeSteps << " Fwd Plan Horizon (s) :  " << p.planHorizonSec << " Time Step : " << p.deltaT << " Num Time Steps per control derivation : " << p.numSimFwdStepsPerThd << "\n";
		out << "# State RB's" << p.numStateBodies << " size of stateRBIdxs : " << p.stateRBIdxs.size() << " Idxs : (";
		for (int i = 0; i < p.stateRBIdxs.size(); ++i) { out << ((i == 0) ? "" : ",") << p.stateRBIdxs[i]; } out << ")\n";
		out << "H Effort Sds : (" << p.highEffortSds[0] << "," << p.highEffortSds[1] << "," << p.highEffortSds[2] << "," << p.highEffortSds[3] << ")\tLow Effort Sds : (" << p.lowEffortSds[0] << "," << p.lowEffortSds[1] << "," << p.lowEffortSds[2] << "," << p.lowEffortSds[3] << ")\n";
		out << "H Effort FMax : (" << p.highEffortFMax[0] << "," << p.highEffortFMax[1] << "," << p.highEffortFMax[2] << ")\tLow Effort FMax : (" << p.lowEffortFMax[0] << "," << p.lowEffortFMax[1] << "," << p.lowEffortFMax[2] << ")\n";
		out << "Resample Threshold : " << p.resampleThreshold << " Uniform Bias : " << p.uniformBias << " Pose Prior Std : " << p.posePriorStd << " State Kernel Scale : " << p.stateKernelScale << " GBP Reg lambda : " << p.gbpRegularization << " Jerk Std : " << p.jerkSd << "\n";
		out << "Max Control Value : "<<p.maxCntlVal<<" Control Prior SD Multiplier : " << p.cntrlPrSDMult << "FMax SD Multiplier" << p.fMaxSDMult << "\n";
		out << "Base Std Scale : " << p.baseStdScale << " Foot Std Scale : " << p.footStdScale << " Knee Std Scale : " << p.kneeStdScale << " Arm Std Scale : " << p.armStdScale << " Mut Scale : " << p.mutationScale << "\n";
		out << "Motor FMax : " << p.motorFmax << " Torso MinFMax " << p.torsoMinFMax << " Legs Min FMax " << p.legsMinFMax << " Arms MinFMax " << p.armsMinFMax << "\n";

		out << "Use Gauss Back Prop : " << (p.flags[p.IDX_useGBP] ? "True" : "False") << " Use State Kernel (Q) : " << (p.flags[p.IDX_useStKnStd] ? "True" : "False") << "\n";
		out << "Use Velocity Motors : " << (p.flags[p.IDX_useVelMtr] ? "True" : "False") << " Apply Jerk Cost : " << (p.flags[p.IDX_useJerkCost] ? "True" : "False") << "\n";
		out << "Use Kernel Log Reg (TODO) : " << (p.flags[p.IDX_useKLReg] ? "True" : "False") << "\n";
		return out;
	}

	//void CPBPParams::setParamValFromXML(const std::string& _name, double _vald, int _vali, bool _valb, std::string& _valstr) {
	//	if (_name.compare("numSamples") == 0) { numSamples = _vali;			return; }
	//	if (_name.compare("numStateBodies") == 0) { numStateBodies = _vali;        return; }
	//	if (_name.compare("numCntrlFlim") == 0) { numCntrlFlim = _vali;        return; }
	//	if (_name.compare("numSimFwdStepsPerThd") == 0) { numSimFwdStepsPerThd = _vali;        return; }
	//	if (_name.compare("idxInWorld") == 0) { idxInWorld = _vali;        return; }

	//	if (_name.compare("skelFileName") == 0) { skelFileName = std::string(_valstr);	return; }
	//	if (_name.compare("skelType") == 0) { skelType = std::string(_valstr);	return; }

	//	if (_name.compare("planHorizonSec") == 0) { planHorizonSec = _vald;        return; }
	//	if (_name.compare("maxCntlVal") == 0) { maxCntlVal = _vald;        return; }

	//	if (_name.compare("lowPosePriorStd") == 0) { lowEffortSds[0] = _vald;        return; }
	//	if (_name.compare("lowControlPriorStd") == 0) { lowEffortSds[1] = _vald;        return; }
	//	if (_name.compare("lowControlDiffPriorStd") == 0) { lowEffortSds[2] = _vald;        return; }
	//	if (_name.compare("lowControlDiffDiffPriorStd") == 0) { lowEffortSds[3] = _vald;        return; }

	//	if (_name.compare("highPosePriorStd") == 0) { highEffortSds[0] = _vald;        return; }
	//	if (_name.compare("highControlPriorStd") == 0) { highEffortSds[1] = _vald;        return; }
	//	if (_name.compare("highControlDiffPriorStd") == 0) { highEffortSds[2] = _vald;        return; }
	//	if (_name.compare("highControlDiffDiffPriorStd") == 0) { highEffortSds[3] = _vald;        return; }

	//	if (_name.compare("lowFMaxControlPriorStd") == 0) { lowEffortFMax[0] = _vald;        return; }
	//	if (_name.compare("lowFMaxControlDiffPriorStd") == 0) { lowEffortFMax[1] = _vald;        return; }
	//	if (_name.compare("lowFMaxControlDiffDiffPriorStd") == 0) { lowEffortFMax[2] = _vald;        return; }

	//	if (_name.compare("highFMaxControlPriorStd") == 0) { highEffortFMax[0] = _vald;        return; }
	//	if (_name.compare("highFMaxControlDiffPriorStd") == 0) { highEffortFMax[1] = _vald;        return; }
	//	if (_name.compare("highFMaxControlDiffDiffPriorStd") == 0) { highEffortFMax[2] = _vald;        return; }

	//	if (_name.compare("deltaT") == 0) { deltaT = _vald;  invDelTNumStpsPerThd = 1.0 / (deltaT*numSimFwdStepsPerThd);      return; }
	//	if (_name.compare("resampleThreshold") == 0) { resampleThreshold = _vald;        return; }
	//	if (_name.compare("uniformBias") == 0) { uniformBias = _vald;        return; }
	//	if (_name.compare("gbpRegularization") == 0) { gbpRegularization = _vald;        return; }
	//	if (_name.compare("baseStdScale") == 0) { baseStdScale = _vald;        return; }
	//	if (_name.compare("footStdScale") == 0) { footStdScale = _vald;        return; }
	//	if (_name.compare("kneeStdScale") == 0) { kneeStdScale = _vald;        return; }
	//	if (_name.compare("armStdScale") == 0) { armStdScale = _vald;        return; }
	//	if (_name.compare("jerkSd") == 0) { jerkSd = _vald;        return; }
	//	if (_name.compare("stateKernelScale") == 0) { stateKernelScale = _vald;        return; }
	//	if (_name.compare("motorFmax") == 0) { motorFmax = _vald;        return; }
	//	if (_name.compare("mutationScale") == 0) { mutationScale = _vald;        return; }
	//	if (_name.compare("torsoMinFMax") == 0) { torsoMinFMax = _vald;        return; }
	//	if (_name.compare("legsMinFMax") == 0) { legsMinFMax = _vald;        return; }
	//	if (_name.compare("armsMinFMax") == 0) { armsMinFMax = _vald;        return; }

	//	if (_name.compare("useGBP") == 0) { flags[IDX_useGBP] = _valb;        return; }
	//	if (_name.compare("useVelMtr") == 0) { flags[IDX_useVelMtr] = _valb;        return; }
	//	if (_name.compare("useJerkCost") == 0) { flags[IDX_useJerkCost] = _valb;        return; }
	//}

}
