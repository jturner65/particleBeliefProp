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

#include "CPBPHandler.h"
#include "ControlPBP.h"
#include "dart/dart.h"
#include "dart/utils/SkelParser.h"
#include "dart/simulation/World.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/Joint.h"

#include "BalanceMotionGoal.h"

#include <memory>
#include <thread>


using namespace AaltoGames;

namespace cPBPropApp {
	CPBPHandler::CPBPHandler(dart::dynamics::SkeletonPtr _skel, dart::simulation::WorldPtr _world, std::shared_ptr<CPBPParams> _cp) :
		pbp(), cp(_cp), masterSimCntxt(nullptr), main_skel(_skel), main_world(_world), numSimSteps(0), numTotalFrames(0), sampleThreads(0),
		currEffortLvl(0), stateKernelStd(), numGoals(1), goals(1), numThdsHardware(std::thread::hardware_concurrency())
	{
		// std::thread::hardware_ concurrency() function returns the number of threads that can run concurrently for a given execution.
		std::cout << "This Machine can handle " << numThdsHardware << " threads without overhead.";
		numThdsHardware = cp->numSamples;		//2x # of native threads
		std::cout << " TODO : All " << cp->numSamples << " samples will be spread evenly amongst " << numThdsHardware << " threads." << "\n";

		numDofs = main_skel->getNumDofs();
		numCntrl = numDofs - 6;										//ignore root dof control
		numStateDim = cp->numStateBodies * 6 + 3;						//  w/6 bodies this is  39 : these are vector for up + 6 * # rigid body : 3-vec location and 3-vec velocity
		numCntrlFlimDim = numCntrl + cp->numCntrlFlim; 				//increased amount for FMAX values (forcelimits), remove 1st 6 dof values ->should be 35
		initMinMaxAngles();
		std::cout << "CPBPHandler Constructor" << "\n";
	}
	CPBPHandler::~CPBPHandler() {}

	void CPBPHandler::initMinMaxAngles() {
		////PBP uses min, max angles, angle ranges and angle std scales  for Cu matrix in posterior generation
		//defaults
		minAngles.setConstant(numCntrl, -.5 * M_PI);
		maxAngles.setConstant(numCntrl, .5 * M_PI);
		angleRanges.setConstant(numCntrl, M_PI);
		angleStdScales.setConstant(numCntrl, cp->baseStdScale);

		//if not using defaults : 
		//hardcoded since not read in from skeleton file (changed in DART 5?)

		//dof names and idx's
			//[0]	"j_pelvis_rot_x"
			//[1]	"j_pelvis_rot_y"
			//[2]	"j_pelvis_rot_z"
			//[3]	"j_pelvis_pos_x"
			//[4]	"j_pelvis_pos_y"
			//[5]	"j_pelvis_pos_z"

			//[6]	"j_thigh_left_z"
			//[7]	"j_thigh_left_y"
			//[8]	"j_thigh_left_x"

			//[9]	"j_shin_left"

			//[10] "j_heel_left_1"
			//[11] "j_heel_left_2"
			//[12] "j_toe_left"

			//[13] "j_thigh_right_z"
			//[14] "j_thigh_right_y"
			//[15] "j_thigh_right_x"

			//[16] "j_shin_right"

			//[17] "j_heel_right_1"
			//[18] "j_heel_right_2"
			//[19] "j_toe_right"

			//[20] "j_abdomen_1"
			//[21] "j_abdomen_2"
			//[22] "j_spine"

			//[23] "j_head_1"
			//[24] "j_head_2"

			//[25] "j_scapula_left"
			//[26] "j_bicep_left_z"
			//[27] "j_bicep_left_y"
			//[28] "j_bicep_left_x"
			//[29] "j_forearm_left"
			//[30] "j_hand_left"

			//[31] "j_scapula_right"
			//[32] "j_bicep_right_z"
			//[33] "j_bicep_right_y"
			//[34] "j_bicep_right_x"
			//[35] "j_forearm_right"
			//[36] "j_hand_right"

		double hPi = M_PI * .5;
		minAngles << //start at IDX 6
			-hPi, -.7, -.4,				//left thigh z,y,x min
			-2.4, -.75, -.6,			//left knee, left heel 1, 2, 
			-.6, 						//left toe

			-hPi, -.7, -1,				//right thigh z,y,x min
			-2.4, -.75, -.9,			//right knee, right heel 1,2
			-.6, 						//right toe

			-.7, -hPi, -1,				//abdomen 1, 2, spine
			-1.1, -.6, 					//head 1,2, 

			-.5,						//scap Left,
			-hPi, -1.2, -.1,			//left bicep z,y,x
			0, -hPi,					//left forearm,left hand,

			-.4,						// scap Right,
			-hPi, -1.2, -M_PI,			//right bicep z,y,x
			0, -1;						//right forearm,  right hand

		maxAngles <<
			hPi, .7, 1,					//left thigh z,y,x max
			0, .75, .9,					//left knee, left heel 1, left heel 2
			.6,							//left toe max

			hPi, .7, .4,				//right thigh z,y,x max
			0, .75, .6,					//right knee, right heel 1,2
			.6,							//right toe max

			.7, .6, 1,					//abdomen 1, 2, spine
			.75, .6,					//head 1,2, 

			.4, 						//scap Left,
			hPi, 1.2, M_PI,				//left bicep z,y,x
			2.5, 1, 					//left forearm, left hand, 

			.5,							//scap Right,
			hPi, 1.2, .1,				//right bicep z,y,x
			2.5, hPi;					//right forearm, right hand

		for (int i = 0; i < numCntrl; ++i) {
			angleRanges[i] = maxAngles[i] - minAngles[i];
		}
		//subtracting 6 because ignoring root dofs
		int armIDX[10] = { 26, 27, 28, 29, 30, 32, 33, 34, 35, 36 };	//dof idx of arm dofs -> sub 6 for angleStdScales idx
		int kneeIDX[2] = { 9, 16 };											//dof idx of knee dofs -> sub 6 for angleStdScales idx
		int footIDX[6] = { 10,11,12, 17, 18, 19 };							//dof idx of foot dofs -> sub 6 for angleStdScales idx
		for (int i = 0; i < 10; ++i) {			angleStdScales[armIDX[i] - 6] = cp->armStdScale;		}
		for (int i = 0; i < 2; ++i) {			angleStdScales[kneeIDX[i] - 6] = cp->kneeStdScale;		}
		for (int i = 0; i < 6; ++i) {			angleStdScales[footIDX[i] - 6] = cp->footStdScale;		}
		angleRngStdProd = cp->cntrlPrSDMult * angleRanges.cwiseProduct(angleStdScales);				//precomp this product for recomp sampling params
	}

	void CPBPHandler::initCPBPHandler() {
		buildMasterCntxt();
		std::cout << "CPBPHandler : Master Context built\n";
		buildSimCntxts();
		std::cout << "CPBPHandler : Sim Contexts built\n";
		//buildCntxtLaunchers();
		std::cout << "CPBPHandler : Context Launchers Not built - 1 thread per contxt\n";
	}

	//set up all context-related data
	void CPBPHandler::buildCntxt(std::shared_ptr<SimContext> cntxt, bool isMstr, int sampID) {
		cntxt->cp = cp;
		if (!isMstr) {
			Eigen::Vector3d tmpSkelLoc(0, 0, 0);
			std::cout << "ID : " << sampID << " Loc : " << tmpSkelLoc(0) << ", " << tmpSkelLoc(1) << ", " << tmpSkelLoc(2) << "\n";
			cntxt->skelLocInWorld = tmpSkelLoc;
		}
		cntxt->initStateControlVars(numStateDim, numCntrlFlimDim, cp->stateRBIdxs, minAngles, maxAngles, angleRanges);
		if (cp->flags[cp->IDX_useVelMtr]) { cntxt->setSkelVelJoint(); std::cout << "use vel motors for cntxt : " << cntxt->name << "\n";}//sets joints to be velocity joints
		else {								cntxt->setSkelServoJoint(); std::cout << "use servo motors for cntxt : " << cntxt->name << "\n"; }
		cntxt->initDebugArrays(isMstr);
	}

	//initialize master context
	void CPBPHandler::buildMasterCntxt() {
		masterSimCntxt = std::allocate_shared<SimContext>(Eigen::aligned_allocator <SimContext>(), main_skel, main_world, "Main");
		buildCntxt(masterSimCntxt, true,-1);
	}

	//initialize all virtual walker skeletons - need numSamples # of skeletons/worlds for particle belief prop
	void CPBPHandler::buildSimCntxts() {
		std::vector<dart::simulation::WorldPtr> tmpSimWorlds(cp->numSamples);
		std::vector<std::shared_ptr<SimContext>> tmpSkelCntxts(cp->numSamples);
		Eigen::Vector3d gravity(0.0, -9.81, 0.0);
		std::stringstream ss;
		//dart::dynamics::SkeletonPtr skel;
		for (int i = 0; i < cp->numSamples; ++i) {																		//1 skel per sample
			tmpSimWorlds[i] = dart::utils::SkelParser::readWorld(cp->skelFileName);
			tmpSimWorlds[i]->setGravity(gravity);
			tmpSimWorlds[i]->setTimeStep(cp->deltaT);
			dart::dynamics::SkeletonPtr skel = tmpSimWorlds[i]->getSkeleton(cp->idxInWorld);
			ss.str("");
			ss << "ThreadCTXT:" << i;
			tmpSkelCntxts[i] = std::allocate_shared<SimContext>(Eigen::aligned_allocator <SimContext>(), skel, tmpSimWorlds[i], ss.str());
			buildCntxt(tmpSkelCntxts[i], false, i);
		}
		mFwdSimWorld = tmpSimWorlds[0];
		mFwdSimCntxts = tmpSkelCntxts;
	}//initSkels

	//init context launchers to hold appropriate # of walkers per thread, 
	//with appropriate # of launchers based on hardware architecture - 
	//evenly spread all samples among all cntxt launchers
	//void CPBPHandler::buildCntxtLaunchers() {
	//	std::vector<std::shared_ptr<CntxtLauncher>> tmpLaunchers(numThdsHardware);
	//	int countPerThd = (int)ceil(cp->numSamples / numThdsHardware), currCount = 0;
	//	std::vector<std::shared_ptr<SimContext>>::const_iterator first, last;

	//	int numCntxtLaunchers = numThdsHardware < cp->numSamples ? numCntxtLaunchers - 1 : cp->numSamples - 1;		//make sure we never make more cntxt launchers than we need
	//		for (int i = 0; i < numCntxtLaunchers; ++i) {
	//		first = mFwdSimCntxts.begin() + currCount;
	//		last = mFwdSimCntxts.begin() + currCount + countPerThd;
	//		std::vector<std::shared_ptr<SimContext>> newVec(first, last);
	//		tmpLaunchers[i] = std::allocate_shared<CntxtLauncher>(Eigen::aligned_allocator <CntxtLauncher>(), newVec, newVec.size());
	//		currCount += countPerThd;
	//	}
	//	first = mFwdSimCntxts.begin() + currCount;
	//	last = mFwdSimCntxts.end();
	//	std::vector<std::shared_ptr<SimContext>> newVec(first, last);
	//	tmpLaunchers[numThdsHardware - 1] = std::allocate_shared<CntxtLauncher>(Eigen::aligned_allocator <CntxtLauncher>(), newVec, newVec.size());
	//	mCntxtLaunchers = tmpLaunchers;
	//
	//}//initCntxtLaunchers

	//currently just have balance goal, TODO add support for multiple, online customizable/modifiable goals
	void CPBPHandler::buildGoals(int _numGoals, std::vector<int>& _numSubG) {
		numGoals = _numGoals;
		goals.resize(numGoals);
		for (int i = 0; i < numGoals; ++i) {
			//goals[i] = std::allocate_shared<BalanceMotionGoal>(Eigen::aligned_allocator <BalanceMotionGoal>(), _numSubG[i], cp->upperBodyRBIdxs);
			goals[i] = std::make_shared<BalanceMotionGoal>(_numSubG[i], cp->upperBodyRBIdxs);
			//goals[i] = std::make_shared<BalanceMotionGoal>(_numSubG[i]);//OLD
			goals[i]->init();
		}
		//add more goals here
		std::cout << numGoals << (numGoals == 1 ? " Goal made " : " Goals made \n");
		std::cout << "CPBPHandler : Goals built\n";
	}

	//initialize goals to use master context's rest pose as rest pose
	void CPBPHandler::initGoals() {
		for (int i = 0; i < numGoals; ++i) {
			goals[i]->setRestState(masterSimCntxt);
			goals[i]->refresh();
		}
		//add more goals here
		std::cout << numGoals << (numGoals == 1 ? " Goal initialized from mstr cntxt " : " Goals initialized from mstr cntxt \n");
	}
	//set this pose as rest pose for all walkers, and re-init goals to hold this pose as rest pose
	//call once, right before first iteration, when main walker hits ground
	void CPBPHandler::setMasterPoseAsRestState(bool useCurrMstrState) {
		std::cout << "Set Master State as Rest State : Use curr Mstr State as Rest State : "<<(useCurrMstrState ? " Yes " : " No ") <<" \n" << masterSimCntxt->buildStateStr_Debug(masterSimCntxt->currState) <<"\n";
		if (useCurrMstrState) {
			masterSimCntxt->getDartSkelState(masterSimCntxt->s, Eigen::Vector3d(0,0,0));
			masterSimCntxt->setStateAsRestState(masterSimCntxt->currState, false, false);
		}
		for (int i = 0; i < cp->numSamples; i++) { mFwdSimCntxts[i]->setStateAsRestState(masterSimCntxt->restState, true, false); }
		initGoals();
	}

	//initialize pbp solver - initialize limits on control, mean, etc. -> masterSimCntxt needs to be made before this is called
	void CPBPHandler::initCPBP() {
		numSimSteps = 0;																				//set # of passed simSteps to be 0
		numTotalFrames = 0;
		//init control prior means and stds
		controlMean.setZero(numCntrlFlimDim);														//set to zero to denote the average motor control being 0, since we have it symmetric extend/contract
		minControl.setConstant(numCntrlFlimDim, -(cp->maxCntlVal));
		maxControl.setConstant(numCntrlFlimDim, cp->maxCntlVal);

		controlPriorStd.setZero(numCntrlFlimDim);
		controlDiffPriorStd.setZero(numCntrlFlimDim);
		controlDiffDiffPriorStd.setZero(numCntrlFlimDim);
		if (0 != cp->stateKernelScale) { stateKernelStd.setZero(numStateDim); }
		else { stateKernelStd.resize(0); }				//use size== 0 to denote not using stateKernelStd

		recompSampleParams();		//this sets up the above
		
		pbp.init(cp->numSamples, cp->numFwdTimeSteps, numStateDim, numCntrlFlimDim, minControl.data(), maxControl.data(), controlMean.data(), controlPriorStd.data(), controlDiffPriorStd.data(), controlDiffDiffPriorStd.data(), cp->mutationScale, cp->stateKernelScale == 0 ? NULL : stateKernelStd.data());
		//set further params: portion of "no prior" samples, resampling threshold, whether to use the backwards smoothing pass, and the regularization of the smoothing pass
		pbp.setParams(cp->uniformBias, cp->resampleThreshold, cp->flags[cp->IDX_useGBP], cp->gbpRegularization);
		std::cout << "C-PBP Initialized " << "\n";
	}

	void CPBPHandler::iterateCPBP() {		//signal the start of new C-PBP iteration
		//std::vector<thread> sampleThreads(0);										//1 thread per context

		recompSampleParams();		//computes posePriorStdev as well as all control stds
		pbp.setSamplingParams(controlPriorStd.data(), controlDiffPriorStd.data(), controlDiffDiffPriorStd.data(), cp->mutationScale, cp->stateKernelScale == 0 ? NULL : stateKernelStd.data());
		pbp.setParams(cp->uniformBias, cp->resampleThreshold, cp->flags[cp->IDX_useGBP], cp->gbpRegularization);
		double tmpCost = 0, varLevel = 0, maxVarLvl = 0;

		//use currstate (full dof pos/vel) instead?
		pbp.startIteration(true, masterSimCntxt->PBPState.data());
		numTotalFrames++;
		//std::cout << "start iteration : " <<numTotalFrames<< "\n";
		//simulate forward 
		for (int k = 0; k < cp->numFwdTimeSteps; k++) {
			sampleThreads.clear();
			refreshAllGoals();														//refresh goal bounds - for variance - set up low and high variance values to be min and max variance of last phase			
			pbp.startPlanningStep(k);												//signal the start of a planning step
			//set states of all contexts - need to be done for all contexts before control is derived and before threads are built
			if (0 == k) {
				for (int i = 0; i < cp->numSamples; i++) {
					//copy master sim context into each sample
					mFwdSimCntxts[i]->updateInitialSimContext(i, masterSimCntxt);
				}
			}//save state of all samples/contexts, setting first step's values to match master sim cntxt
			else {
				for (int i = 0; i < cp->numSamples; i++) {
					int prevIdx = pbp.getPreviousSampleIdx(i);
					mFwdSimCntxts[i]->updateSimContext(k, i, mFwdSimCntxts[prevIdx]);
				}
			}//save state of all samples/contexts  and update from previous
			//get control via C-PBP

			//TODO each contxt launcher should have N/#threads contxts, which are run in sequence.  N is determined by processor capabilities
			for (int i = 0; i < mFwdSimCntxts.size(); i++) {
				////launch this in a thread
				sampleThreads.push_back(std::move(std::thread(&SimContext::deriveControl, mFwdSimCntxts[i], std::ref(pbp))));
				//sampleThreads.push_back(std::thread(&SimContext::deriveControl, mFwdSimCntxts[i], std::ref(pbp)));
				//  sampleThreads.push_back(std::thread(&CntxtLauncher::launchContexts, mCntxtLaunchers[i], std::ref(pbp)));
				//mFwdSimCntxts[i]->deriveControl(pbp);
			}
			std::for_each(sampleThreads.begin(), sampleThreads.end(), std::mem_fn(&std::thread::join));

			//evaluate against goals maybe eventually fold this into deriveControl (each thread), once we can guarantee motiongoal's thread safety 
			for (int i = 0; i < cp->numSamples; ++i) {
					tmpCost = 0;
					varLevel = 0, maxVarLvl = 0;
					for (int gidx = 0; gidx < goals.size(); ++gidx) {
						tmpCost += goals[gidx]->calcFrmCost(mFwdSimCntxts[i], varLevel, cp->deltaT * k);
						maxVarLvl = (varLevel > maxVarLvl ? varLevel : maxVarLvl);
					}
					mFwdSimCntxts[i]->updateVarAndCostPerStep(tmpCost, maxVarLvl);
				}
			//}
			//goals evaluated now, update pbp
			for (int i = 0; i < cp->numSamples; ++i) {
				mFwdSimCntxts[i]->updateOptResults(pbp);
			}			
			pbp.endPlanningStep(k);																		//signal the end of the planning step. this normalizes the state costs etc. for the next step
		}	
		pbp.endIteration();																				//signal the end of an iteration. this also executes the backwards smoothing pass
		//deploy the best control found to mstrSimCntxt
		//can get best cost/fitness here - this is PBP's take on the cost
		double optSqCost = pbp.getBestTrajectoryCost();
		//double bestFitness = exp(-.5*optSqCost);
		//double bestCost = optSqCost/(1.0*cp->numFwdTimeSteps);
		int bestIdx = pbp.getBestSampleLastIdx();
		currEffortLvl = mFwdSimCntxts[bestIdx]->varLevel / (1.0*cp->numFwdTimeSteps);
		//std::cout << "OptSqCost : " << optSqCost << " avg cost : " <<bestCost<< " best idx : " << bestIdx << " curr Eff Lvl : " << currEffortLvl << " # fwd prediction steps : "<< cp->numFwdTimeSteps<<"\n";
		currEffortLvl = (currEffortLvl > 0 ? ((currEffortLvl < 1) ? currEffortLvl : 1) : 0);				//must remain bound between 0 and 1
		pbp.getBestControl(0, masterSimCntxt->control.data());												//put best control in master-context's control vector
		if (masterSimCntxt->_flags[masterSimCntxt->debug]) {
			masterSimCntxt->debugStepStartState.push_back(masterSimCntxt->currState);		//state that this cntxt is starting at
			masterSimCntxt->debugPrevIdx.push_back(-1);			
		}
		//if (cp->flags[cp->IDX_useVelMtr]) { 
			masterSimCntxt->s->setCommands(masterSimCntxt->commandIdxs, masterSimCntxt->control.head(numCntrl)); 
		//}
		//else {
		//	//make control vector, move to appropriate idxs in skel (do not set root dofs)
		//	Eigen::VectorXd ActualControl;								//when we decrease size of control vector by 6, use this to accomodate passing complete control
		//	ActualControl.setZero(numDofs);
		//	ActualControl.tail(numCntrl) << masterSimCntxt->control.head(numCntrl);	//do not use ControlPBP to derive root control
		//	masterSimCntxt->s->setForces(ActualControl);
		//}
		if (masterSimCntxt->_flags[masterSimCntxt->debug]) {
			masterSimCntxt->debugStepState.push_back(masterSimCntxt->currState);
			masterSimCntxt->debugStepCntrl.push_back(masterSimCntxt->control);
		}
		//forward step mastersimcntxt's world in stepMstrCntxtGetCost()
	}//iterateCPBP

	//step master context forward, and calculate cost of motion
	double CPBPHandler::stepMstrCntxtGetCost(bool beginCPBP) {
		masterSimCntxt->ownWorld->step();			//master context world - step forward, called by MyWindow::timeStepping
		masterSimCntxt->updateStateControl();
		double mainCntxtCost = 0;
		if (beginCPBP) {//we've started cpbp and iterated at least 1 time
			mainCntxtCost += masterSimCntxt->calcJerkUpdVel();//calculate jerk cost, also populate reduced state

			double varLevel = 0;
			for (int i = 0; i < numGoals; ++i) { mainCntxtCost += goals[i]->calcFrmCost(masterSimCntxt, varLevel, masterSimCntxt->elapsedSimTime); }
		}
		return mainCntxtCost;
	}
	 
	//recomputes sample params/priors every iteration - currEffortLvl is interpolant between bounds for sds 
	void CPBPHandler::recompSampleParams() {
		double t = currEffortLvl,
			stKernStdVal = 0.5*cp->stateKernelScale;
		std::vector<double> currentSds = lerpV4(cp->lowEffortSds, t, cp->highEffortSds);				//currentSds must never be 0 in any dimension
		std::vector<double> currFMax = lerpV4(cp->lowEffortFMax, t, cp->highEffortFMax);

		cp->posePriorStd = currentSds[0];														

		controlPriorStd.head(numCntrl) = currentSds[1] * angleRngStdProd;
		controlDiffPriorStd.head(numCntrl) = currentSds[2] * angleRngStdProd;
		controlDiffDiffPriorStd.head(numCntrl) = currentSds[3] * angleRngStdProd;

		minControl.tail<5>() << cp->torsoMinFMax, cp->armsMinFMax, cp->armsMinFMax, cp->legsMinFMax, cp->legsMinFMax;
		maxControl.tail<5>() = Eigen::VectorXd().setConstant(5, cp->motorFmax);
		Eigen::VectorXd diffCtl = maxControl.tail<5>() - minControl.tail<5>();
		
		controlPriorStd.tail<5>() = currFMax[0] * maxControl.tail<5>();
		controlDiffPriorStd.tail<5>() = currFMax[1] * diffCtl;
		controlDiffDiffPriorStd.tail<5>() = currFMax[2] * diffCtl;
		if (0 != cp->stateKernelScale) {
			stateKernelStd.setConstant(numStateDim, stKernStdVal);
		}
	}//recompSampleParams

	//reset all shared pointers, to make way for rebuilding
	void CPBPHandler::resetAllCntxts() {
		masterSimCntxt.reset();
		mFwdSimWorld->reset();
		for (int i = 0; i < mFwdSimCntxts.size(); ++i) { mFwdSimCntxts[i].reset(); }
		for (int i = 0; i < mCntxtLaunchers.size(); ++i) { mCntxtLaunchers[i].reset(); }
	}

	std::vector<double> CPBPHandler::accumulateVals() {		return cp->accumulateVals();}//grab all locals that are modifiable by UI or used elsewhere
	void CPBPHandler::distributeVals(std::vector<double>& vals) {cp->distributeVals(vals);}

	//get values to send to UI
	void CPBPHandler::sendValsToUI() {
		std::vector<double> res = accumulateVals();
		for (int i = 0; i < UI->MyPgUISldr.size(); ++i) { UI->MyPgUISldr[i]->setSliderVal((float)(res[i])); }
		UI->MyPgUIBtn[4]->isChecked = cp->flags[cp->IDX_useGBP];
	}

	void CPBPHandler::getValsFromUI() {
		//get all slider vals
		std::vector<double> vals;
		for (int i = 0; i < UI->MyPgUISldr.size(); ++i) { vals.push_back(UI->MyPgUISldr[i]->getCurValue()); }
		distributeVals(vals);
		//sendToContexts(vals);
		sendValsToUI();					//refresh UI with values - verify that all values are being set properly
	}

	void CPBPHandler::resetUIWithDefVals() { cp->resetValues();		sendValsToUI(); }
	void CPBPHandler::saveUIVals() {		std::cout << "Not implemented yet" << "\n";	}//save current settings to disk

	void CPBPHandler::buildUI() {
		//UIObjIdx
		//TODO : read this in from XML? Port from knowledge/kinect proj
		const int numMainBtns = 4;
		const int numCBox = 1;
		const int numSliderCaps = 29;								//# of slider/caption pairs - 1 per data entry value

		int numObjs = numMainBtns + (numSliderCaps * 2) + (numCBox * 2);		//# big buttons + num (caps + sliders) + cap + button for checkboxes //+ 1 for debug caption
		std::string buttonNames[numMainBtns] = { std::string("Set Values"), std::string("Reset Values"), std::string("Save Values"), std::string("Re Init CPBP") };

		std::string sldrValNamesL[15] = { "Horizon Sec",
			"Leff PosePr Std", "Leff CPr Std", "Leff CDPr Std", "Leff CDDPr Std",							//6
			"Leff FMax CPr Std", "Leff FMax CDPr Std", "Leff FMax CDDPr Std",											//13
			"Resample Thresh", "Uniform Bias", "GPB Reg", "Base Std Scale",		//26
			"Foot Std Scale", "Knee Std Scale", "Arm Std Scale" };																			//31

		double sldrMinValsL[15] = { .1, 
			.0001, .0001, .001, .001,
			.0001, .0001, .0001,
			.001, .0001, .0001, .1,
			.01, .01, .01 };
		double sldrMaxValsL[15] = { 5.0,
			10, 10, 10, 10,
			10, 10, 10,
			.999, .9999, .9999, 10,
			50, 50, 50 };

		std::string sldrValNamesR[14] = { "Timestep",
			"Heff PosePr Std", "Heff CPr Std", "Heff CDPr Std", "Heff CDDPr Std",						
			"Heff FMax CPr Std", "Heff FMax CDPr Std", "Heff FMax CDDPr Std",							
			"Jerk Std", "Motor Fmax", "Ctrl Mut Std", 
			"TorsoMinFMax", "LegsMinFMax", "ArmsMinFMax"//add slider for fMaxSDMult
			};																		

		double sldrMinValsR[14] = { .001,
			.001, .001, .001, .001,
			.001, .001, .001,
			.01, 50, .001,
			1, 1, .1};
		double sldrMaxValsR[14] = { .05,
			20, 20, 20, 20,
			20, 20, 20,
			2.0, 500, .999, 
			100, 75, 10};

		UI->numObjs += numObjs;								//# of ui objects
		std::vector<int> objType;								//what type each object is : 0 : button, 1 : slider, 2 : caption, 3 : textbox
		std::vector<std::string> objLabels;							//object labels
		std::vector<std::vector<int>> objXY_WH;
		std::vector<std::vector<float>> objClr;

		objType.resize(numObjs);								//what type each object is : 0 : button, 1 : slider, 2 : caption, 3 : textbox
		objXY_WH.resize(numObjs);
		objLabels.resize(numObjs);
		objClr.resize(numObjs);
		int idx = 0;
		std::stringstream ss;
		float initY = 10, _stOff = 10, sliderLen = 20, sliderWide = 15, widBuf = 48, hWidBuf = widBuf*.5f, qWidBuf = widBuf*.25f, tqWidBuf = hWidBuf + qWidBuf, col2 = UI->sldrBarLen + sliderLen + _stOff + _stOff,
			capWide = 30, yBuffer1 = widBuf + capWide, btnWide = col2, btnHigh = 25, btnBuf = 5, stY = initY + (tqWidBuf*numSliderCaps*.5f) + hWidBuf, wTBDisp = btnWide + btnBuf, hTBDisp = btnHigh + btnBuf;
		for (int i = 0; i < 2; ++i) {
			ss.str("");		ss << buttonNames[i];
			std::vector<float> v1 = { i*wTBDisp, stY, btnWide, btnHigh, .75f, .75f, .75f, 1 };
			UI->initUIObj(idx++, ss.str(), 0, v1, objType, objLabels, objXY_WH, objClr);
			ss.str("");		ss << buttonNames[i + 2];
			std::vector<float> v2 = { i*wTBDisp, stY + hTBDisp, btnWide, btnHigh, .75f, .75f, .75f, 1 };
			UI->initUIObj(idx++, ss.str(), 0, v2, objType, objLabels, objXY_WH, objClr);
		}

		for (int i = 0; i < 15; ++i) {
			std::vector<float> v1 = { _stOff+qWidBuf, initY + (tqWidBuf*i), sliderWide, sliderLen, .85f, .85f, .85f, 1 };
			UI->initUIObj(idx++, "", 1, v1, objType, objLabels, objXY_WH, objClr);
			ss.str("");		ss << sldrValNamesL[i];
			std::vector<float> v2 = { _stOff + hWidBuf, initY + (tqWidBuf*i) + capWide, capWide, capWide, .25f, .25f, .25f, 1 };
			UI->initUIObj(idx++, ss.str(), 2, v2, objType, objLabels, objXY_WH, objClr);
			if (i == 14) { 
				ss.str("");		ss << "Enable";
				std::vector<float> vb = { _stOff + col2 + sliderWide, initY + (tqWidBuf*i), sliderLen, sliderLen, .75f, .75f, .75f, 1 };
				UI->initUIObj(idx++, ss.str(), 0, vb, objType, objLabels, objXY_WH, objClr);
				ss.str("");		ss << "Enable Gaussian Back Prop ";
				std::vector<float> vCap = { _stOff + col2 + sliderWide + hWidBuf, _stOff + initY + (tqWidBuf*i), capWide, capWide, .25f, .25f, .25f, 1 };
				UI->initUIObj(idx++, ss.str(), 2, vCap, objType, objLabels, objXY_WH, objClr);
			}
			else {
				std::vector<float> v3 = { _stOff + col2 + sliderWide, initY + (tqWidBuf*i), sliderWide, sliderLen, .85f, .85f, .85f, 1 };
				UI->initUIObj(idx++, "", 1, v3, objType, objLabels, objXY_WH, objClr);
				ss.str("");		ss << sldrValNamesR[i];
				std::vector<float> v4 = { _stOff + col2 + sliderWide + qWidBuf, initY + (tqWidBuf*i) + capWide, capWide, capWide, .25f, .25f, .25f, 1 };
				UI->initUIObj(idx++, ss.str(), 2, v4, objType, objLabels, objXY_WH, objClr);
			}
		}
		//ss.str("");		ss << "CPBP Debug : ";
		//vector<float> vCapDB = { _stOff + qWidBuf, stY + 2 * hTBDisp + qWidBuf, capWide, capWide, .25f, .25f, .25f, 1 };
		//UI->initUIObj(idx++, ss.str(), 2, vCapDB, objType, objLabels, objXY_WH, objClr);

		//call this after init - sets up all objects
		UI->setUI(objType, objLabels, objXY_WH, objClr);

		UI->width = 2*wTBDisp;
		UI->height = stY + 2 * hTBDisp;// +capWide;

		//set caption for each slider
		for (int i = 0; i < UI->MyPgUISldr.size(); ++i) {
			UI->MyPgUISldr[i]->setCaption(UI->MyPgUICaption[i], UI->MyPgUISldr[i]);
		}
		for (int i = 0; i < UI->MyPgUISldr.size(); i += 2) {
			UI->MyPgUISldr[i]->setSlideMin(sldrMinValsL[i / 2]);
			UI->MyPgUISldr[i]->setSlideRng(sldrMaxValsL[i / 2] - sldrMinValsL[i / 2]);
			if (i / 2 >= 14) { continue; }
			UI->MyPgUISldr[i + 1]->setSlideMin(sldrMinValsR[i / 2]);
			UI->MyPgUISldr[i + 1]->setSlideRng(sldrMaxValsR[i / 2] - sldrMinValsR[i / 2]);
		}
		for (int i = 0; i < UI->MyPgUIBtn.size(); ++i) {
			if (i < numMainBtns) { UI->MyPgUIBtn[i]->flags[UI->MyPgUIBtn[i]->UIobjIDX_CtrLbl] = true; }
			else { UI->MyPgUIBtn[i]->flags[UI->MyPgUIBtn[i]->UIobjIDX_CtrLbl] = false;  UI->MyPgUIBtn[i]->isCheckBox = true; UI->MyPgUIBtn[i]->isChecked = cp->flags[i - numMainBtns]; }//initCBState[] holds the booleans that correspond to checkboxes.  
		}//center button captions on first 3 buttons, set rest to be checkboxes
	}
	
	void CPBPHandler::reinitCPBP() {//reset skeletons to initial hit-the-ground state, reset cpbp - use new values
		std::vector<double> vals;
		for (int i = 0; i < UI->MyPgUISldr.size(); ++i) { vals.push_back(UI->MyPgUISldr[i]->getCurValue()); }
		double newDelT = vals[1], newPredHoriz = vals[0];
		distributeVals(vals);
		//sendToContexts(vals);
		//update the cpbp-related values that need to be resent to init - NOTE this can change the nature of the sim results
		cp->setPlanHorizAndFwdTS(vals[0], newDelT);

		sendValsToUI();					//refresh UI with values to visually verify that all values are being set properly
		//use master start state to restart - needs to be original rest state from master sim
		masterSimCntxt->initStateControlVars(numStateDim, numCntrlFlimDim, cp->stateRBIdxs, minAngles, maxAngles, angleRanges);
		masterSimCntxt->initDebugArrays(true);
		this->masterSimCntxt->setStateFromSkelState(this->masterSimCntxt->restState, Eigen::Vector3d(0, 0, 0));
		for (int i = 0; i < this->mFwdSimCntxts.size(); ++i) {
			this->mFwdSimCntxts[i]->initStateControlVars(numStateDim, numCntrlFlimDim, cp->stateRBIdxs, minAngles, maxAngles, angleRanges);
			this->mFwdSimCntxts[i]->setStateAsRestState(this->masterSimCntxt->restState, true, false);
		}
		initCPBP();
	}

	//go through the UI objects, find what has been clicked
	bool CPBPHandler::handleBtnClick() {
		bool result = false;
		for (int i = 0; i < UI->MyPgUIBtn.size(); i++) {
			if (UI->MyPgUIBtn[i]->isClicked()) {
				std::string onclick = UI->MyPgUIBtn[i]->getOnClick();
				std::cout << "Button Clicked ID:" << i << "|" << UI->MyPgUIBtn[i]->getLabel() << " onclick = " << onclick << "\n";
				switch (i) {
					case 0: {getValsFromUI(); return true; }//"Send Values to handler from UI",
					case 1: {saveUIVals(); return true; }//"Save Values",
					case 2: {resetUIWithDefVals(); return true; }//"Reset Values for ui and handler",
					case 3: {reinitCPBP(); return true; }//"Re Init Sim",
					default: {
						if ((UI->MyPgUIBtn[i]->isCheckBox) && (i < cp->numFlags + 4)) {//turn on/off sim control from check box
							cp->flags[i - 4] = UI->MyPgUIBtn[i]->isChecked;				//currently only for turning on or off gbp
							return true;
						}						
						return false; }
				}
			}//if click
		}//for each button
		return false;
	}

	std::ostream& operator<<(std::ostream& out, CPBPHandler& hndl) {//for dbug output TODO

		return out;
	}
}
