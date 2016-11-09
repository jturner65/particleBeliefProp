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

#ifndef APPS_PARTICLEBELIEFPROP_SimContext_H_
#define APPS_PARTICLEBELIEFPROP_SimContext_H_

#include <vector>
#include <deque>
#include <string>
#include "ControlPBP.h"
#include "CPBPParams.h"
#include "MotionGoal.h"
#include "dart/dynamics/DegreeOfFreedom.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/simulation/World.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/constraint/ConstraintSolver.h"
#include "dart/collision/CollisionDetector.h"

#include <memory>

#include <Eigen/Dense>
#include <Eigen/StdVector>
using namespace std;
using namespace AaltoGames;

namespace dart {
    namespace dynamics {
        class BodyNode;
        class Skeleton;
        class Joint;
		class DegreeOfFreedom;
        class Marker;
    }  // namespace dynamics
}  // namespace dart

namespace cPBPropApp {
	class CPBPParams;
	class CPBPHandler;

	/** Helper types for STL containers with fixed size Eigen memory allocators - from MRPT. */
	//template <class TYPE1, class TYPE2 = TYPE1>
	template <class TYPE1, class TYPE2 = TYPE1>
	struct aligned_containers {
		typedef std::pair<TYPE1, TYPE2> pair_t;
		typedef std::vector<TYPE1, Eigen::aligned_allocator<TYPE1> > vector_t;
		typedef std::vector<std::vector<TYPE1, Eigen::aligned_allocator<TYPE1>>> vec_vec_t;
		typedef std::deque<TYPE1, Eigen::aligned_allocator<TYPE1> > deque_t;
		typedef std::list<TYPE1, Eigen::aligned_allocator<TYPE1> > list_t;
		typedef std::map<TYPE1, TYPE2, std::less<TYPE1>, Eigen::aligned_allocator<std::pair<const TYPE1, TYPE2> > > map_t;
		typedef std::multimap<TYPE1, TYPE2, std::less<TYPE1>, Eigen::aligned_allocator<std::pair<const TYPE1, TYPE2> > > multimap_t;
	};
	
	class SimContext {

	public:
		typedef cPBPropApp::aligned_containers<dart::collision::Contact>::vector_t dbCntct;
		typedef cPBPropApp::aligned_containers<dart::collision::Contact>::vec_vec_t dbCntctList;
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		explicit SimContext(dart::dynamics::SkeletonPtr _skel, dart::simulation::WorldPtr _w, std::string _name);		
		//copy ctor
		SimContext(const SimContext& _other);
		////move ctor
		//SimContext(SimContext&& _other);								//move constructor

		virtual ~SimContext() {}

		void setInitVals();
		//initialize all state and control variables for this context - called at the beginning of every fwd sim cycle
		void initStateControlVars(int _nStDim, int _numCtrlFlimDim, std::vector<int>& stBodyIdxs, const Eigen::Ref<const Eigen::VectorXd>& minAngles, const Eigen::Ref<const Eigen::VectorXd>& maxAngles, const Eigen::Ref<const Eigen::VectorXd>& _angleRanges);
		//init arrays used to hold state, control and contact info for debugging
		void initDebugArrays(bool mstrCntxt);

		//called every sim update
		void updateStateControl(); 
		//initial update of contxt vars from master sim cntxt
		void updateInitialSimContext(int sample, std::shared_ptr<SimContext> mstrCntxt);
		//update all context-related variables like sample id, step idx, variance level, world state, etc.
		void updateSimContext(int step, int sample, std::shared_ptr<SimContext> prevCtxt);
		//called from cntxt launcher - use pbp to derive control vector, and apply that control
		void deriveControl(ControlPBP& pbp);
		//apply the derived control values to this context's skeleton - dbIdx is the idx used to store trajectory info - for sim cntxts it is whichever fwd predict step we are on, for mstrcntxt it is which ever frame we're on
		void applyControl(const Eigen::Ref<const Eigen::VectorXd>& cntrl);
		//update COM and com velocity
		void updateCOMandCVel();
		//calculate cop if in contact with an object - need a separate COP for each potential object in contact with (i.e. steps)? 
		Eigen::Vector3d calcCop(const Eigen::Vector3d& _com);
		//determine if this skeleton is in contact with the ground - set in COP calculation
		bool isTouchingGround() {
			return _flags[inCntct];		//expand to be contact specifically between feet and ground TODO
		}
		//called on each goal evaluation - passed goal cost and variance output from goal eval for every per-step goal
		void updateVarAndCostPerStep(double _cst, double _var) {
			curCostSum += _cst;
			maxStepVarLvl = (_var > maxStepVarLvl ? _var : maxStepVarLvl);
		}
		//increase variance and update pbp with cost results from goals calculation
		//per step prior mean and std are optional, describe pose prior to be used for control proposal and control cost derivation
		void updateOptResults(ControlPBP& pbp) {
			varLevel += maxStepVarLvl;
			computeStateVector();
			curCostSum += calcJerkUpdVel();																	//will be updated for each possible goal calculation
			pbp.updateResults(sampleIDX, control.data(), PBPState.data(), curCostSum, perStepPriorMean.data(), perStepPriorStd.data());
			if (_flags[debug]) {		debugCosts[stepIDX] = curCostSum;			}
		}

		//trigger by UI interaction?
		Eigen::VectorXd& getDartSkelState(dart::dynamics::SkeletonPtr _skel, const Eigen::Vector3d& disp) { currState = _skel->getState(); return currState; }
		void updateSkelKinState() { updateSkelKinState(currState); }
		void updateSkelKinState(const Eigen::Ref<const Eigen::VectorXd>& skelState) {	s->setState(skelState); s->computeForwardKinematics(true, true, false); }	//if (stepWorld) { ownWorld->step(); }
		void setStateFromSkelState(const Eigen::Ref<const Eigen::VectorXd>& _dss, const Eigen::Vector3d& disp) { currState = _dss; updateSkelKinState(); }
		//skelState needs to be numDofs*2 in size, result of getState() call on skeleton
		void setStateAsRestState(const Eigen::Ref<const Eigen::VectorXd>& skelState, bool setAsCurrState, bool setStationary) {
			if (setStationary) { //remove velocity component of passed state
				restState.setZero(numDofs * 2);
				restState.head(numDofs) = skelState.head(numDofs);			//set all skelState vels to be 0 for reststate
			}
			else {				restState = skelState;			}
			if (setAsCurrState) {
				currState = restState;				
				updateSkelKinState();
			}
		}//setStateAsRestState
		//set values when they change in UI
		//void setDTimeAndJerkVar();
		//from unity code - modfies how wrists are handled upon contact with solid surface
		void overrideWristAnglesOnImpacts();
		//calculate the cost for the jerk on the jerk bodies (reduced state) (cost for changing acceleration) and update the history of velocities for future jerk cost calcs
		double calcJerkUpdVel();
		//build (reduced) state vector of rbs position and velocity
		void computeStateVector();
		//void setTimeStep(double _ts);
		//set this cntxt's skeleton to be velocity joints
		void setSkelVelJoint();
		//set this cntxt's skeleton to be servo joints
		void setSkelServoJoint();

		inline Eigen::Vector3d getRBPos(int idx) { return nodes[idx]->getCOM(); }
		//call at the end of every prediction step, to build display string of debug results
		string debugPidxOut();
		string debugCstOut();
		inline string buildStrFromFloat(double val, const char* fmt ) {
			char buf[MAX_BUF];
			stringstream ss;
			sprintf(buf, fmt, val);
			ss << buf;
			return ss.str();// label = ss.str();
		}//buildLabel

		//copy assignment - needed?
		SimContext& operator=(const SimContext& _other) {	
			//Eigen::VectorXd
			control = _other.control;									//control,
			PBPState = _other.PBPState;								//PBPState,
			perStepPriorMean = _other.perStepPriorMean;				//perStepPriorMean,
			perStepPriorStd = _other.perStepPriorStd;					//perStepPriorStd,
			currState = _other.currState;								//currState,					
			restState = _other.restState;								//restState,					
			angleRanges = _other.angleRanges;							//angleRanges = 
			//std::vector<int>
			stateRBIdxs = _other.stateRBIdxs;
			handRBIdxs = _other.handRBIdxs;
			handDofIdxs = _other.handDofIdxs;
			handUpDir = _other.handUpDir;
			//std::vector<Eigen::Vector3d>
			sampleVel = _other.sampleVel;
			prevSampleVel = _other.prevSampleVel;
			//double
			jerkVar = _other.jerkVar;									//jerkVar,			
			jerkCost = _other.jerkCost;								//jerkCost,			
			curCostSum = _other.curCostSum;							//curCostSum,			
			prevCost = _other.prevCost;								//prevCost,			
			posePriorStd = _other.posePriorStd;						//posePriorStd,		
			maxStepVarLvl = _other.maxStepVarLvl;						//maxStepVarLvl,
			varLevel = _other.varLevel;								//varLevel,			
			prevVarLevel = _other.prevVarLevel;
			mass = _other.mass;										//mass
			elapsedSimTime = _other.elapsedSimTime;					//elapsedSimTime;

			ownWorld = _other.ownWorld;								//dart::simulation::World * ownWorld;			 				
			cdSolver = _other.cdSolver;								//dart::constraint::ConstraintSolver* cdSolver					
			s = _other.s;												//dart::dynamics::SkeletonPtr s;                                     
			root = _other.root;										//dart::dynamics::BodyNode* root;                                  
			nodes = _other.nodes;										//std::vector<dart::dynamics::BodyNode*> nodes;                    
			joints = _other.joints;									//std::vector<dart::dynamics::Joint*> joints;                      
			dofs = _other.dofs;										//std::vector<dart::dynamics::DegreeOfFreedom*> dofs;              
			markers = _other.markers;									//std::vector<dart::dynamics::Marker*> markers;                    
			//int 
			numNodes = _other.numNodes;								//numNodes = 		
			numJoints = _other.numJoints;								//numJoints = 
			numDofs = _other.numDofs;									//numDofs = 
			numCtrl = _other.numCtrl;									//numCtrl = <--# of actual controls
			numFLim = _other.numFLim;									//numFLim,		
			numCtrlFlimDim = _other.numCtrlFlimDim;								//numCtrlFlimDim,
			numStateBodies = _other.numStateBodies;						//numStateBodies,	
			numStateDim = _other.numStateDim;								//numStateDim,		
			numMarkers = _other.numMarkers;							//numMarkers,
			sampleIDX = _other.sampleIDX;								//sampleIDX,			
			prevSampleIDX = _other.prevSampleIDX;						//prevSampleIDX,		
			stepIDX = _other.stepIDX;									//stepIDX,		
			//std::vector<std::string> :nodeNames ; jointNames ;dofNames ;markerNames;
			nodeNames = _other.nodeNames;
			jointNames = _other.jointNames;
			dofNames = _other.dofNames;
			markerNames = _other.markerNames;
			//Eigen::Vector3d COM,				COMLinVel,		COPCalcPt,			COP;
			COM = _other.COM;
			COMLinVel = _other.COMLinVel;
			COP = _other.COP;
			COPCalcPt = _other.COPCalcPt;
			//std::vector<Eigen::Vector3d>
			COPList = _other.COPList;
			//std::vector<bool>
			_flags = _other._flags;
			//string
			name = _other.name;
			//std::vector<Eigen::VectorXd>
			debugStepStartState = _other.debugStepStartState;			
			debugStepCntrl = _other.debugStepCntrl;		
			debugStepState = _other.debugStepState;		
			debugStepContacts = _other.debugStepContacts;		
			debugCosts = _other.debugCosts;
			return *this;
		}
		//for debugging purposes, copy a contact from cop calc
		inline void copyCntct(const dart::collision::Contact& src, dart::collision::Contact& dest) {

			dest.point << src.point;
			dest.normal << src.normal;
			dest.force << src.force;

			dest.bodyNode1 = src.bodyNode1;
			dest.bodyNode2 = src.bodyNode2;
			dest.shape1 = src.shape1;
			dest.shape2 = src.shape2;
			dest.penetrationDepth = src.penetrationDepth;
		}

		std::string buildStrFromEigenXd(const Eigen::Ref<const Eigen::VectorXd>& vec) { int sz = vec.size(); stringstream ss; for (int i = 0; i < sz - 1; ++i) { ss << buildStrFromFloat(vec(i), "%.8f") << ","; }ss << buildStrFromFloat(vec(sz - 1), "%.8f");	return ss.str(); }
		std::string buildStateStr_Debug(const Eigen::Ref<const Eigen::VectorXd>& vec) {
			int sz = vec.size(), dofSize = dofNames.size();
			stringstream ss;
			for (int i = 0; i < dofSize; ++i) { ss << i << ":" << dofNames[i] << " pos : " << vec(i) << endl; }
			ss << endl;
			for (int i = 0; i < dofSize; ++i) { ss << i << ":" << dofNames[i] << " vel : " << vec(i + dofSize) << endl; }
			return ss.str();
		}

		friend std::ostream& operator<<(std::ostream& out, SimContext& cntxt); 

	public ://variables
		std::vector<Eigen::VectorXd> debugStepState,								//for debug purposes, this is a list of all previous step states of the skeleton,
			debugStepStartState,													// and the start state of the skeleton at the beginning of each fwd predict step
			debugStepCntrl;															//debug previous controls
		//std::vector<std::vector<dart::collision::Contact>> debugStepContacts;		//per step lists of contacts

		dbCntctList debugStepContacts;	//per step lists of contacts - aligned

		std::vector<double> debugCosts;												//each step cost for each step of the fwd prediction

		std::vector<int> debugPrevIdx;												//previous idx
		std::string name;															//name of this context - will be "main" for main context

		Eigen::VectorXd
			control,
			perStepPriorMean,
			perStepPriorStd,
			PBPState,																	//reduced state for PBP internal calcs
			angleRanges,
			currState,																	//the current pose and velocity of this context's skeleton - saved after controls are updated and world is stepped, used for goals and for next sample
			restState;																	//target rest pose and velocity for prior (should set rest vel to be 0, only will use head of this for cpbp pose targeting currently)

		std::vector<size_t> commandIdxs;												//idx's for commands for us to set

		std::vector<int>
			stateRBIdxs,													//idx's of rb's evaluated to calculate jerk cost/reduced state calculation
			handRBIdxs,														//idx's of hand bodies, for custom wrist handling on impact overrideWristAnglesOnImpacts
			handDofIdxs,													//idx's of first dof for wrist angle for each hand
			handUpDir;														//1 or -1, depending on hand - direction in hand's space, for up motion at wrist

		std::vector<Eigen::Vector3d>
			sampleVel,
			prevSampleVel;										//last two velocities for every rb in control, for jerk calculation and to pass between contexts for each new sample

		//general skeleton/simulation-related terms
		dart::simulation::WorldPtr ownWorld;												//simulation world owning this context/skeleton
		dart::constraint::ConstraintSolver* cdSolver;
		dart::dynamics::SkeletonPtr s;                                                    //skeleton
		dart::dynamics::BodyNode* root;                                                 //root of the skeleton
		std::vector<dart::dynamics::BodyNode*> nodes;                                   //refs to body nodes
		std::vector<dart::dynamics::Joint*> joints;                                     //refs to joints
		std::vector<dart::dynamics::DegreeOfFreedom*> dofs;                             //refs to dofs
		std::vector<dart::dynamics::Marker*> markers;                                   //refs to markers
		Eigen::Vector3d COM,															//center of mass
			COMLinVel,																	//com velocity
			COP,																		//COP of contact with ground - defined as the point where the resultant moment of all contact forces is 0
			COPCalcPt,																	//point to calculate cop around - set once as simcntxt is loaded, needs to be on ground plane
			skelLocInWorld;																//to put skeletons in same world
		std::vector<Eigen::Vector3d> COPList;											//may be in contact with multiple objects, idx 0 is always ground, 1 cop per object contacting

		double 
			jerkVar,																	//jerk variance - sq of std
			jerkCost,																	//cost of jerk penalty - for display purposes
			curCostSum,																	//current cost - set after apply control, evaulating local cost functions, but not goals
			prevCost,																	//previous cost
			posePriorStd,																//pose prior std
			maxStepVarLvl,
			varLevel,																	//variance level
			prevVarLevel,
			mass,
			elapsedSimTime;

		int numNodes, numJoints, numDofs, numMarkers,
			numCtrl,																	//# of controls used = numDofs - 6
			numFLim,																	//# of force limit types for this control scheme - default is 5
			numCtrlFlimDim,																//# of cntrl used + number of force limits = numCtrl + numFLim
			numStateBodies,																//# of body nodes in state
			numStateDim,																	//dimension of state vector 3 + 6*numStateBodies
			sampleIDX,																	//current sample this context owns
			prevSampleIDX,																//the sample index corresponding to the previous sample to this one, in linked trajectories
			stepIDX;																	//current step of forward sim prediction being executed		

		std::vector<std::string> nodeNames, jointNames, dofNames, markerNames;
		//const int pelvisRBIdx = 0, lftHeelRBIdx = 3, rtHeelRBIdx = 7, headRBIdx = 11, lftToeRBIdx = 4, rtToeRBIdx = 8, lftHandRBIdx = 18, rtHandRBIdx = 19;

		//std::vector<dart::collision::Contact> contacts
		std::shared_ptr<cPBPropApp::CPBPParams> cp;
		std::vector<bool> _flags;														//flags about world and skeleton state
		static const int inCntct = 0;															//if the skeleton is in contact with another physical body
		static const int copInit = 1;															//cop has been initialized to legitimiate initial value
		static const int calcJerkCost = 2;															//incorporate jerk cost
		static const int debug = 3;																//process debug data

		const int numFlags = 4;

		//from Forces Acting on a Biped Robot : COP-ZMP by Sardain and Bessonnet
		//Given a contact force c at point P, we can split it into 2 components:
		//1) cn(P) the normal force denoting the local pressure force 
		//2) ct(P) and a tangential force  denoting local friction force
		//the resultant force and moment corresponding to the normal forces at a point Q is :
		//Rcn = sum(cn(P)) dot n
		//Mcn_q = sum(cn(P)dot(Q,P)) x n
		//and the resultant force and moment correspoding to the tangent forces at point Q is :
		//Rct = sum(ct(P)), where Rct dot n == 0
		//Mct_q = sum((Q,P) x ct(P))
		//therefore the COP is a) the point where the Normal moment Mcn_COP is 0 or b) the frictive moment is parallel to the contact normal

		//ZMP;//TODO : ZMP is same as COP unless both feet lie on different surfaces	//zero moment point - defined as the point on the ground where the gravity and inertia-based tipping moment is 0 (tipping moment is moment tangent to ground)
		//from Forces Acting on a Biped Robot : COP-ZMP by Sardain and Bessonnet
		//Given the Moment M @ any point Q : M = (Q,COM) x mg - (Q,COM) x ma - HdotCOM
		//mg : mass * grav; ma : mass * acceleration of COM; HdotG : angular momentum of COM
		//Then the ZMP is the point at the intersection of the axis where the moment is parallel to the normal n of the contact forces to the surface of that contact
		//ie: M = (ZMP,COM) x mg - (ZMP,COM) x ma - HdotCOM where M x n = 0

	};//SimContext
}//namespace 
#endif  // APPS_PARTICLEBELIEFPROP_SimContext_H_
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
