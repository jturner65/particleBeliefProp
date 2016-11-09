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
#include "ControlPBP.h"
#include "CPBPHandler.h"
#include "dart/common/Console.h"
#include "dart/math/Helpers.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/DegreeOfFreedom.h"
#include "dart/dynamics/Joint.h"
#include "dart/dynamics/Marker.h"
#include "dart/dynamics/PointMass.h"
#include "dart/dynamics/SoftBodyNode.h"
#include "dart/collision/CollisionDetector.h"
#include <thread>
#include <mutex>
#include <memory>
//#include <mutex>

using namespace std;
using namespace Eigen;
using namespace dart;
using namespace dynamics;
using namespace AaltoGames;

namespace cPBPropApp {	

	SimContext::SimContext(SkeletonPtr _skel, dart::simulation::WorldPtr _wd, std::string _name) :
		s(_skel), ownWorld(_wd), jerkVar(0), jerkCost(0), handRBIdxs(2), handDofIdxs(2), handUpDir(2), name(_name), cp(nullptr), 
		numNodes(0), numJoints(0), numDofs(0), numMarkers(0), mass(0), COM(0, 0, 0), COP(-1, -1, -1), COPList(1), COPCalcPt(0, 0, 0), COMLinVel(0, 0, 0), elapsedSimTime(0), _flags(1), posePriorStd(), 
		debugStepState(0), debugStepCntrl(0), debugStepStartState(0), debugCosts(0), debugPrevIdx(0), debugStepContacts(0), commandIdxs(0),
		curCostSum(0), prevCost(0), maxStepVarLvl(0),  varLevel(0), prevVarLevel(0), skelLocInWorld(0,0,0),
		control(), PBPState(), perStepPriorMean(), perStepPriorStd(), 
		//fullDimControl(), minAngles(), maxAngles(), angleStdScales(),newControlState(),
		angleRanges(), restState(), currState()
	{	
		cout << "SimCntxt Cnstrctr : " << name << endl;
	}

	//copy ctor
	SimContext::SimContext(const SimContext& _o) :
		s(_o.s), ownWorld(_o.ownWorld), jerkVar(_o.jerkVar), jerkCost(_o.jerkCost), handRBIdxs(_o.handRBIdxs), handDofIdxs(_o.handDofIdxs), handUpDir(_o.handUpDir), name(_o.name), cp(_o.cp),
		numNodes(_o.numNodes), numJoints(_o.numJoints), numDofs(_o.numDofs), numMarkers(_o.numMarkers), mass(_o.mass), COM(_o.COM), COP(_o.COP), COPList(_o.COPList), COPCalcPt(_o.COPCalcPt), COMLinVel(_o.COMLinVel), 
		elapsedSimTime(_o.elapsedSimTime), _flags(_o._flags), posePriorStd(_o.posePriorStd), curCostSum(_o.curCostSum), prevCost(_o.prevCost), skelLocInWorld(_o.skelLocInWorld), commandIdxs(_o.commandIdxs),
		debugStepState(_o.debugStepState), debugStepContacts(_o.debugStepContacts), debugStepStartState(_o.debugStepStartState), debugStepCntrl(_o.debugStepCntrl), debugCosts(_o.debugCosts), debugPrevIdx(_o.debugPrevIdx),
		maxStepVarLvl(_o.maxStepVarLvl), varLevel(_o.varLevel), prevVarLevel(_o.prevVarLevel), control(_o.control), PBPState(_o.PBPState), perStepPriorMean(_o.perStepPriorMean), perStepPriorStd(_o.perStepPriorStd),
		angleRanges(_o.angleRanges), restState(_o.restState), currState(_o.currState)
	
	{
		cout << "SimCntxt Copy Cnstrctr : " << _o.name << endl;
	}

	//these should only change if component of skeleton is destroyed and remade - fast access to components of skeleton
	void SimContext::setInitVals() {
		mass = s->getMass();
		root = s->getRootBodyNode();
		//cstrntSlv = ownWorld->getConstraintSolver();
		_flags.resize(numFlags, false);
		_flags[debug] = true;
		COPList.resize(1, Eigen::Vector3d(0, 0, 0));
		//nodes
		numNodes = s->getNumBodyNodes();
		nodeNames = vector<string>(numNodes, "");
		nodes = vector<BodyNode*>(numNodes, NULL);
		for (int i = 0; i < numNodes; ++i) {		nodes[i] = s->getBodyNode(i);			nodeNames[i] = nodes[i]->getName();		}

		//joints
		numJoints = s->getNumBodyNodes();
		jointNames = vector<string>(numJoints, "");
		joints = vector<Joint*>(numJoints, NULL);
		for (int i = 0; i < numJoints; ++i) {		joints[i] = nodes[i]->getParentJoint();			jointNames[i] = joints[i]->getName();		}

		//dofs
		numDofs = s->getNumDofs();
		dofNames = vector<string>(numDofs, "");
		dofs = vector<DegreeOfFreedom*>(numDofs, NULL);
		for (int i = 0; i < numDofs; ++i) {			dofs[i] = s->getDof(i);			dofNames[i] = dofs[i]->getName();		}

		//controls for PBP
		numCtrl = numDofs - 6;							//do not use first 6 dofs (root) for control
		commandIdxs.resize(numCtrl);
		for (int i = 6; i < numDofs; ++i) {
			commandIdxs[i-6] = i;
		}
		//hand objects for ray-casting (TODO ?)
		handRBIdxs[0] = 15;																	//left hand rigid body idx in nodes array
		handRBIdxs[1] = 19;																	//right hand
		handDofIdxs[0] = 30;																//left hand dof idx in dofs array - wrists are single dof dofs. 
		handDofIdxs[1] = 36;																//right hand
		handUpDir[0] = 1;																	//left hand up TODO check this
		handUpDir[1] = -1;																	//right hand up

		sampleIDX = -1;																			//current sample this context owns
		prevSampleIDX = -1;																		//the sample index corresponding to the previous sample to this one, in linked trajectories
		stepIDX = -1;
		//markers
		numMarkers = 0;
		for (vector<BodyNode*>::const_iterator it = nodes.begin(); it != nodes.end(); ++it) {
			numMarkers += (*it)->getNumMarkers();
		}
		markerNames = vector<string>(numMarkers, "");
		markers = vector<Marker*>(numMarkers, NULL);
		int tmpIDX = 0;
		//markers idx in array need to be the same as index as in the skeleton

		for (vector<BodyNode*>::const_iterator it = nodes.begin(); it != nodes.end(); ++it) {
			for (int i = 0; i < (*it)->getNumMarkers(); ++i) {
				markers[tmpIDX] = (*it)->getMarker(i);
				markerNames[tmpIDX] = markers[tmpIDX]->getName();
				tmpIDX++;
			}
		}

		restState.setZero(numDofs * 2);													//rest pose is first 1/2 -should be set when skeleton is at "rest" position - wait for initial contact with ground
		getDartSkelState(s,skelLocInWorld);
		cout << name << " : setInitVals Done " << endl;
	}

	void SimContext::initDebugArrays(bool mstrCntxt) {
		debugStepState.clear();
		debugPrevIdx.clear();
		debugStepCntrl.clear();
		debugStepStartState.clear();
		debugCosts.clear();
		debugStepContacts.clear();
		if (!mstrCntxt) {
			debugStepContacts.resize(cp->numFwdTimeSteps);
			debugStepState.resize(cp->numFwdTimeSteps);
			debugStepCntrl.resize(cp->numFwdTimeSteps);
			debugCosts.resize(cp->numFwdTimeSteps);
			debugStepStartState.resize(cp->numFwdTimeSteps);
			debugPrevIdx.resize(cp->numFwdTimeSteps);
		}
	}
	//run one time
	void SimContext::initStateControlVars(int _nStDim, int _numCtrlFlimDim, std::vector<int>& _stBodyIdxs, const Eigen::Ref<const Eigen::VectorXd>& minAngles, const Eigen::Ref<const Eigen::VectorXd>& maxAngles, const Eigen::Ref<const Eigen::VectorXd>& _angleRanges) {
		setInitVals();
		int dofIdx;
		for (int i = 0; i < numCtrl; ++i) {
			dofIdx = i + 6;
			dofs[dofIdx]->setPositionLimits(minAngles[i], maxAngles[i]);				//set joint limits based on min/max values
		}

		for (int i = 0; i < numJoints; ++i) {joints[i]->setPositionLimited(true);}
		
		angleRanges = _angleRanges;
		ownWorld->setTimeStep(cp->deltaT);												//initialize these for master sim cntxt
		jerkVar = cp->jerkSd*cp->jerkSd;
		_flags[calcJerkCost] = cp->flags[cp->IDX_useJerkCost];

		numCtrlFlimDim = _numCtrlFlimDim; numStateDim = _nStDim; numStateBodies = _stBodyIdxs.size(); numFLim = numCtrlFlimDim - numCtrl;
		control.setZero(numCtrlFlimDim);
		PBPState.setZero(numStateDim);
		perStepPriorMean.setZero(numCtrlFlimDim);
		perStepPriorStd.setZero(numCtrlFlimDim);

		stateRBIdxs = vector<int>(numStateBodies);
		for (int i = 0; i < numStateBodies; ++i) {
			stateRBIdxs[i] = _stBodyIdxs[i];
		}

		sampleVel = vector<Eigen::Vector3d>(numStateBodies, Eigen::Vector3d(0, 0, 0));
		prevSampleVel = vector<Eigen::Vector3d>(numStateBodies, Eigen::Vector3d(0, 0, 0));

		cdSolver = ownWorld->getConstraintSolver();

		Eigen::Vector3d tmpWCom = ownWorld->getSkeleton(0)->getCOM();						//get center of mass of ground - needs to be -surface- of ground plane.  query for ground thickness
		double thickness = ownWorld->getSkeleton(0)->getBodyNode(0)->getCollisionShape(0)->getBoundingBoxDim()(1);
		COPCalcPt(0) = 150; COPCalcPt(1) = tmpWCom(1)+(.5 * thickness); COPCalcPt(2) = 150;				//set point where moments should be generated around for cop calculation - make it on the ground of the world
		updateCOMandCVel();																	//initializing context
		computeStateVector();

		cout << name << " : initStateControlVars Done " << endl;
	}


	void SimContext::updateInitialSimContext(int sample, std::shared_ptr<SimContext> mstrCntxt) {
		sampleIDX = sample;																			//current sample representing this context 
		stepIDX = 0;
		varLevel = 0;																				//variance inited to 0 every time step, increased every forward predicted control derivation from prev cntxt's value
		posePriorStd = cp->posePriorStd;
		prevSampleIDX = -1;																			//requires special handling for master context?
		elapsedSimTime = mstrCntxt->elapsedSimTime;
		getDartSkelState(mstrCntxt->s, Eigen::Vector3d(0,0,0));																//copy master context's state to this ctxt currstate, to update this skel in derive control below, in thread
		ownWorld->setTimeStep(cp->deltaT);
		jerkVar = cp->jerkSd*cp->jerkSd;
		if (_flags[debug]) {
			//initialize debug constructs
			initDebugArrays(false);
			Eigen::VectorXd tmpC, tmpS;
			tmpC.setZero(numCtrl);
			tmpS.setZero(numDofs * 2);			
			//clear all
			for (int i = 0; i < cp->numFwdTimeSteps; ++i) {
				dbCntct tmpCntctList(0);
				debugStepContacts[i] = tmpCntctList;
				debugStepStartState[i] = tmpS;
				debugStepState[i] = tmpS;
				debugStepCntrl[i] = tmpC;
				debugCosts[i] = 0;
				debugPrevIdx[i] = -1;
			}
			//end init debug constructs
		}
		//need to update physics/skeleton too
		for (int i = 0; i < numStateBodies; ++i) {//probably not needed - can do this with lines above
			//save skel reduced state from end of previous sample prevCtxt to this context. 
			sampleVel[i] = mstrCntxt->sampleVel[i];									//set in jerk calc
			prevSampleVel[i] = mstrCntxt->prevSampleVel[i];
		}
	}//updateInitialSimContext

	//set initial values of context before new sample gets processed - BEFORE OPTIMIZE/CONTROL DERIVATION, in SINGLE THREAD MODE <--NOT THREAD SAFE (relies on previous context for info)
	//this includes moving values from previous, connected(to this sample) sample's context into appropriate locations in this context - this is from last step(?)
	void SimContext::updateSimContext(int step, int sample, std::shared_ptr<SimContext> prevCntxt) {
		sampleIDX = sample;																				//current sample this context owns
		stepIDX = step;
		prevVarLevel = prevCntxt->varLevel;
		prevSampleIDX = prevCntxt->sampleIDX;															//requires special handling for master context?
		getDartSkelState(prevCntxt->s, prevCntxt->skelLocInWorld);										//copy prev context's state to this ctxt currstate, to update this skel in derive control below, in thread
		for (int i = 0; i < numStateBodies; ++i) {//probably not needed - can do this with lines above
			//save skel state from end of previous sample prevCtxt to this context. 
			sampleVel[i] = prevCntxt->sampleVel[i];									//set in jerk calc
			prevSampleVel[i] = prevCntxt->prevSampleVel[i];
		}

	}//updateNewContext

	//use c-pbp to derive control - called runControlStep in Unity
	//this function is called in threading code
	void SimContext::deriveControl(ControlPBP& pbp) {
		curCostSum = 0;
		maxStepVarLvl = 0;
		varLevel = prevVarLevel;

		//update skeleton with dartSkelState, that is built in updateSimContext from prev context's state
		updateSkelKinState();																		//updates skeleton's state in sim world with currState, which was set in updateSimContext	
		//get pose of skeleton and use to calc prior mean							in unity : rig.getCurrentSteppedPoseFromCharacter(ref c.perStepPriorMean.arr); <-- this gets motor angles from previous sample's skeleton - done here in updateSimContext
		//perStepPriorMean.head(numCtrl) = cp->invDelTNumStpsPerThd * (restState.head(numCtrl) - currState.head(numCtrl));		//scalar * vector -- first numDofs vals are pose info -> finite difference derivation of control vel to get us to rest pose
		perStepPriorMean.head(numCtrl) = cp->invDelTNumStpsPerThd * (restState.segment(6, numCtrl) - currState.segment(6, numCtrl));		//scalar * vector -- first numDofs vals are pose info -> finite difference derivation of control vel to get us to rest pose
		perStepPriorStd.head(numCtrl) = (posePriorStd * cp->invDelTNumStpsPerThd) * angleRanges;							    //scalar * vector -- std of velocities of potential velocities
		//TODO: factor in reaching or favoring movement here - set std for arm doing reaching here to be scaled by some factor
		//perStepPriorStd[leftArmIdx] *= armStdScale;
		//perStepPriorStd[rightArmIdx] *= armStdScale;
		perStepPriorMean.tail(numFLim) = Eigen::VectorXd().setZero(numFLim);
		perStepPriorStd.tail(numFLim) = Eigen::VectorXd().setConstant(numFLim, cp->fMaxSDMult * cp->motorFmax);		//fMaxSdMult * motor fmax
		
		//lock guard put in clipped gaussian
		pbp.getControl(sampleIDX, control.data(), perStepPriorMean.data(), perStepPriorStd.data());
		//overrideWristAnglesOnImpacts();			//handle if about to touch ground with hands - need raycasting - from  unity
		//if wanting to do multiple physics steps per opt update, numSimFwdStepsPerThd should be > 1.  need to evaluate per-step goals in here too, so would need to guarantee their thread safety
		//double tmpVarLevel = 0, tmpCost = 0, tmpNonPhysCost = 0;
		if (_flags[debug]) {
			debugStepStartState[stepIDX] = currState;		//state that this cntxt is starting at
			debugPrevIdx[stepIDX] = prevSampleIDX;			//prev cntxt idx to this one
			debugStepCntrl[stepIDX] = control;
		}

		for (int i = 0; i<cp->numSimFwdStepsPerThd; ++i) {//end multiple fwd steps per sample
			applyControl(control);						//NEED TO RECODE IF numSimFwdStepsPerThd ever exceeds 1 - need to keep track of every control application for debug		//sets controls, steps simulation
		}
		//update com, cop and comvel, and current pose, and any other values we want to easily access in cost computation and derive new reduced state vector
		updateStateControl();

		if (_flags[debug]) {
			debugStepState[stepIDX] = currState;
		}
	}//deriveControl

	//apply the derived control values to this context's skeleton
	//called in threading code
	//std::mutex simCntxt_mx;	//global mutex
	void SimContext::applyControl(const Eigen::Ref<const Eigen::VectorXd>& cntrl) {

		//if (cp->flags[cp->IDX_useVelMtr]) {		
			s->setCommands(commandIdxs, cntrl.head(numCtrl)); 
		//}
		//else {										
		//	Eigen::VectorXd ActualControl;								//when we decrease size of control vector by 6, use this to accomodate passing complete control
		//	ActualControl.setZero(numDofs);
		//	ActualControl.tail(numCtrl) << cntrl.head(numCtrl);	//do not use ControlPBP to derive root control
		//	s->setForces(ActualControl);		
		//}
		//if(_flags[debug]) {			
		//	debugStepStartState[dbIdx] = currState;		//state that this cntxt is starting at
		//	debugPrevIdx[dbIdx] = prevSampleIDX;			//prev cntxt idx to this one
		//	debugStepCntrl[dbIdx] = cntrl;
		//}
		//in unity : calls callback function to handle if wanted to generate disturbances, or other last minute interaction before stepping
		//disturb()
		//step the world - perform here only in multi-world simulation
		//std::lock_guard<std::mutex> lock(simCntxt_mx); //lock guard in case multi-theading is causing the issue with contacts
		//if (!cp->flags[cp->IDX_useSingleWorld]) {
		ownWorld->step();
		////update com, cop and comvel, and current pose, and any other values we want to easily access in cost computation and derive new reduced state vector
		//updateStateControl();

		//if (_flags[debug]) {
		//	debugStepState[dbIdx] = currState;
		//}
	}//applyControl
	//set all joints but root joint to be velocity based joints
	void SimContext::setSkelVelJoint() {
		joints[0]->setActuatorType(dart::dynamics::Joint::PASSIVE);
		for (int i = 1; i < numJoints; ++i) { joints[i]->setActuatorType(dart::dynamics::Joint::VELOCITY); }
	}
	void SimContext::setSkelServoJoint() {
		joints[0]->setActuatorType(dart::dynamics::Joint::PASSIVE);
		for (int i = 1; i < numJoints; ++i) { joints[i]->setActuatorType(dart::dynamics::Joint::SERVO); }
	}
	void SimContext::updateStateControl() {
		updateCOMandCVel();
		elapsedSimTime += cp->deltaT;
		//evaluate jerk result
		//curCostSum += calcJerkUpdVel();																	//will be updated for each possible goal calculation
		//computeStateVector();
		getDartSkelState(s, skelLocInWorld);								//sets current local copy of dart skel state vector (dof pos and vel) from skeleton
	}

	void SimContext::updateCOMandCVel() {
		COM = s->getCOM();
		COP = calcCop(COM);
		COMLinVel = s->getCOMLinearVelocity();
	}
	//calculate cop if in contact with an object - need a separate COP for each potential object in contact with? (TODO)

	Eigen::Vector3d SimContext::calcCop(const Eigen::Vector3d& _com) {
		Eigen::Vector3d _cop(_com), avgCntct(0, 0, 0);						//duplicate com for starting point of cop
		if (_flags[copInit]) { _cop = COP; }								// if cop has been inited to legit value already, use that to start with - want to keep legit cop as cop, incase we get no, or bad, new contact info	
		//dart::constraint::ConstraintSolver* cdSolver = ownWorld->getConstraintSolver();
		//dart::collision::CollisionDetector* cd = ownWorld->getConstraintSolver()->getCollisionDetector();			//getting collision detector here because the addition of more colliders may introduce a different collision detector
		dart::collision::CollisionDetector* cd = cdSolver->getCollisionDetector();						//getting collision detector here because the addition of more colliders may introduce a different collision detector
		dart::collision::Contact cntct, tmpCntct;
		int numContacts = cd->getNumContacts();
		dbCntct tmpCntctVec;
		if (0 == numContacts) {
			if (_flags[debug]) {
				if (sampleIDX != -1) { debugStepContacts[stepIDX] = tmpCntctVec; }
				else {					debugStepContacts.push_back(tmpCntctVec);		}
			}
			_flags[inCntct] = false;
			COPList[0] = _cop;
			return _cop;
		}
		
		_flags[inCntct] = true;
		string bn1, bn2;
		//for each contact, check body nodes to see what we are colliding with
		//determine # of bodies we are colliding with, build separate cop for each, with idx0 always being with the ground
		Eigen::Vector3d mSum(0, 0, 0), fSum(0, 0, 0), m(0, 0, 0), surfNorm(0, -1, 0);			//sum of moments and tmp var to hold a moment
		Eigen::Vector3d rotPt(0, 0, 0);
		int numRealCntcts = 0;
		for (size_t k = 0; k < numContacts; k++) {
			cntct = cd->getContact(k);
			if (_flags[debug]) {
				copyCntct(cntct, tmpCntct);
				tmpCntctVec.push_back(std::move(tmpCntct));
			}
			if (0.00000001 > cntct.force.norm()) {			
				continue;		
			}
			avgCntct += cntct.point;
			numRealCntcts++;
			fSum += cntct.force;
			rotPt = cntct.point - COPCalcPt;
			m = rotPt.cross(cntct.force);			//moment around origin is vector from origin to contact point crossed with contact force
			mSum += m;									//COP is point of application of sum of forces to equal sum of moments
			if (k == 0) { surfNorm = cntct.normal; }

			//bn1 = cntct.bodyNode1->getName(), bn2 = cntct.bodyNode2->getName();
			//if (name.compare("Main") == 0) {
			//cout << name << " : Cntct : " << k << " of :" << numContacts << " : " << bn1 << " and " << bn2
			//	<< " | pt : " << cntct.point(0) << "," << cntct.point(1) << "," << cntct.point(2)
			//	<< " | rot Pt : " << rotPt(0) << "," << rotPt(1) << "," << rotPt(2)
			//	<< " | f : " << cntct.force(0) << "," << cntct.force(1) << "," << cntct.force(2)
			//	<< " | n : " << cntct.normal(0) << "," << cntct.normal(1) << "," << cntct.normal(2)
			//	<< " | m = v.cross(f) : " << m(0) << "," << m(1) << "," << m(2)
			//	<< " | mSum : " << mSum(0) << "," << mSum(1) << "," << mSum(2)
			//	<< " | f.n : " << cntct.force.dot(cntct.normal) << endl;
			//}
		}
		if (_flags[debug]) {
			if (sampleIDX != -1) { debugStepContacts[stepIDX] = tmpCntctVec; }
			else {					debugStepContacts.push_back(tmpCntctVec);		}
		}
		//cout << endl;
		//(cop - COPCalcPt) = surface norm x sum of moments / sum of forces dot norm
		//rotPt = (1.0 / (fSum.dot(-surfNorm))) * ((-surfNorm).cross(mSum));
		double tmpRes = (fSum.dot(-surfNorm));
		if ((abs(tmpRes) < .00001) || (numRealCntcts == 0)){
			//cout << "\nNAN : # real cntcts: " << numRealCntcts <<"sIDX : "<<sampleIDX<<"\nCOM: " << COM(0) << "," << COM(1) << "," << COM(2) << endl;
			//cout << "COP: " << _cop(0) << "," << _cop(1) << "," << _cop(2) << "\n" << endl;//COPCalcPt
			//cout << "COPCalcPt: " << COPCalcPt(0) << "," << COPCalcPt(1) << "," << COPCalcPt(2) << "\n" << endl;//COPCalcPt
			//cout << "surfNorm: " << surfNorm(0) << "," << surfNorm(1) << "," << surfNorm(2) << "\n" << endl;
			//cout << "mSum: " << mSum(0) << "," << mSum(1) << "," << mSum(2) << "\n" << endl;
			//cout << "fSum: " << fSum(0) << "," << fSum(1) << "," << fSum(2) << "\n" << endl;
			if (_flags[copInit]) {//use legitimate COP value (not just copied COM
				COPList[0] = COP;			
				return COP;
			}
			//here means we have contacts with 0 force behind them
			_cop = (numRealCntcts != 0 ? avgCntct /= numRealCntcts : COM);
			_cop[1] = COPCalcPt[1];// force cop to be on ground
		}
		else {
			rotPt = (1.0 / tmpRes) * ((-surfNorm).cross(mSum));
			_cop = rotPt + COPCalcPt;// (1.0 / (fSum.dot(-surfNorm))) * ((-surfNorm).cross(mSum));
			_flags[copInit] = true;
		}

		COPList[0] = _cop;
		return _cop;
	}//calcCop

	//part of cost calculation - compute AFTER OPTIMIZE/CONTROL DERIVATION
	double SimContext::calcJerkUpdVel() {
		double rbJkCst;
		jerkCost = 0;
		Eigen::Vector3d tmpVel;
		for (int idx = 0; idx < numStateBodies; ++idx) {
			tmpVel = nodes[stateRBIdxs[idx]]->getLinearVelocity();
			rbJkCst = (-prevSampleVel[idx] + (2 * sampleVel[idx]) - tmpVel).squaredNorm();//Jerk derived through 2nd order finite difference of current, and past 2, velocities
			prevSampleVel[idx] = sampleVel[idx];
			sampleVel[idx] = tmpVel;
			jerkCost += rbJkCst;
		}
		if (!_flags[calcJerkCost]) {	jerkCost = 0; }
		else {							jerkCost /= (numStateBodies * jerkVar); }
		//if () {std::cout<<"Jerk eval on "<<name<<" : jerk Var : " <<jerkVar<< " jerk cost : "<<jerkCost<<"\n";}
		return jerkCost;
	}
	//build this context's reduced state vector - up vector + (pos + vel) for each rbi in stateRBIdxs (in world coords) => only 6 bodies
	//**** PBPState is sent to pbp.updateResults
	void SimContext::computeStateVector() {
		PBPState.head<3>() = (nodes[0]->getWorldTransform().linear().col(1));
		int idx;
		for (int i = 0; i < numStateBodies; ++i) {
			idx = 6 * i;
			//Eigen::Vector3d tmp = nodes[stateRBIdxs[i]]->getCOM();
			//cout << "name : "<< name<<" \t|index :" << i << "stateRBIdxs: " << stateRBIdxs[i] << " obj : " << nodeNames[stateRBIdxs[i]] << " com pos : " << tmp(0) << "," << tmp(1) << "," << tmp(2);
			PBPState.segment(idx + 3, 3) = nodes[stateRBIdxs[i]]->getCOM();
			//tmp = nodes[stateRBIdxs[i]]->getLinearVelocity();
			//cout << " com vel : " << tmp(0) << "," << tmp(1) << "," << tmp(2) << endl;
			PBPState.segment(idx + 6, 3) = nodes[stateRBIdxs[i]]->getLinearVelocity();
		}
		//int breakPoint = 1;//for debug
	}

	//modify control values based upon hands contacting the ground, or about to, or reaching toward something
	void SimContext::overrideWristAnglesOnImpacts() {
		//does dart allow for ray cast intersection finding?  need that here.
		Eigen::Vector3d handPos, handVelDir;
		bool collide = false;
		for (int i = 0; i < 2; ++i) {
			handPos = nodes[handRBIdxs[i]]->getCOM();										//get world position of each hand from DART
			handVelDir = nodes[handRBIdxs[i]]->getLinearVelocity().normalized();			//get velocity of each hand from DART
			//use handPos and handVel to ray cast into world for collision
			//if hit, modify control on hand at hand dof idx's of this->control to pull hand up
			//collide = raycast(handPos, handVelDir, into-the-world, out hitPos, out hitDepth, dt);		//should return if raycast hits scenery during timestep, and where it happens and how deep it penetrates
			if (collide) {
				control[handDofIdxs[i]] = handUpDir[i] * 2 * M_PI;								//modify control if collision imminent
				collide = false;
			}
		}
	}//overrideWristAnglesOnImpacts

	string SimContext::debugCstOut() {//only for sim cntxts, not master cntxt
		if (!_flags[debug]) { return string(""); }
		stringstream ss;
		ss << "c0:" << buildStrFromFloat(debugCosts[0], "%.2f");
		for (int i = 1; i < cp->numFwdTimeSteps; ++i) {
			ss << "-c" << i << ":" << buildStrFromFloat(debugCosts[i], "%.2f");
		}
		return ss.str();
	}

	string SimContext::debugPidxOut() {//only for sim cntxts, not master cntxt
		if (!_flags[debug]) {			return string("");	}
		stringstream ss;
		ss << "p0:" << debugPrevIdx[0];
		for (int i = 1; i < cp->numFwdTimeSteps; ++i) {
			ss << "-p" << i << ":" << debugPrevIdx[i];
		}
		return ss.str();
	}


	std::ostream& operator<<(std::ostream& out, SimContext& cntxt) {
		out << "cntxteton mass : " << cntxt.mass << "| #Nodes : " << cntxt.numNodes << "| #Joints : " << cntxt.numJoints << "| #Dofs : " << cntxt.numDofs << " | #Markers : " << cntxt.numMarkers << endl;
		//nodes  
		for (int i = 0; i < cntxt.numNodes; ++i) {	out << "Node (" << i << ") Name : " << cntxt.nodeNames[i] << endl;}out << "\n";
		for (int i = 0; i < cntxt.numJoints; ++i) {	out << "Joint (" << i << ") Name : " << cntxt.jointNames[i] << endl;}out << "\n";
		//dofs
		for (int i = 0; i < cntxt.numDofs; ++i) {	out << "Dof (" << i << ") Name : " << cntxt.dofNames[i] << endl;}out << "\n";
		//markers
		for (vector<BodyNode*>::const_iterator it = cntxt.nodes.begin(); it != cntxt.nodes.end(); ++it) {
			out << "Markers for Node Name : " << (*it)->getName() << endl;
			for (int i = 0; i < (*it)->getNumMarkers(); ++i) { out << "\t " << i << "th marker on node : " << (*it)->getMarker(i)->getName() << endl; }
			out << "\n";
		}
		return out;
	}
}//namespace cPBPropApp {