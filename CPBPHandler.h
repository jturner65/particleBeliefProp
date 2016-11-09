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
#ifndef APPS_PARTICLEBELIEFPROP_CPBPHandler_H_
#define APPS_PARTICLEBELIEFPROP_CPBPHandler_H_

#include <vector>
#include <deque>
#include <list>
#include <string>
#include "CPBPParams.h"
#include "ControlPBP.h"
#include "SimContext.h"
#include "CntxtLauncher.h"
#include "MotionGoal.h"
#include "dart/dynamics/DegreeOfFreedom.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/simulation/World.h"
#include "MyGuiHandler.h"

#include <memory>
#include <thread>

#include <Eigen/Dense>
#include <Eigen/StdVector>
using namespace std;
using namespace AaltoGames;
namespace cPBPropApp {

	class CPBPHandler {
		//this class handles the calls to the controlPBP algorithm and manages all the sim cntxts
	public:
		CPBPHandler(dart::dynamics::SkeletonPtr _skel, dart::simulation::WorldPtr _world, std::shared_ptr<CPBPParams> _cp);
		virtual ~CPBPHandler();
		//init angle-related quantities for contexts
		void initMinMaxAngles();
		//initialize all cpbp contexts, context launcher, and controlpbp 
		void initCPBPHandler();
		//initialize CPBP algorithm
		void initCPBP();
		//iterate CPBP for each frame of animation
		void iterateCPBP();
		//reset or delete all shared or reg ptrs - use this to reset simulation
		void resetAllCntxts();

		//build (cnstructr) goal list
		void buildGoals(int _numGoals, vector<int>& _numSubG);
		//set either set reststate or current pose as rest pose for all walkers, including master cntxt
		void setMasterPoseAsRestState(bool useCurrMstrSt);
		//step master context forward, and calculate cost of motion
		double stepMstrCntxtGetCost(bool beginCPBP);
		//recompute control prior parameters to be sent to CPBP at beginning of every forward simulation
		void recompSampleParams();

		void getValsFromUI();
		void sendValsToUI();
		vector<double> accumulateVals();
		void distributeVals(vector<double>& vals);
		void resetUIWithDefVals();
		void saveUIVals();
		//event handler for UI components related to this class
		bool handleBtnClick();
		//build the UI construct to modify the values of this class
		void buildUI();
		//void initUIObj(int idx, string label, int type, vector<double>& vals, vector<int>& objType, vector<string>& objLabels, vector<vector<int>>& objXY_WH, vector<vector<double>>& objClr);
		//initialize UI
		void initUI() { buildUI();	sendValsToUI(); }

		void reinitCPBP();
		friend std::ostream& operator<<(std::ostream& out, CPBPHandler& cpbp);
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	private :
		//build initialize context
		void buildCntxt(std::shared_ptr<SimContext> cntxt, bool isMstr, int sampID);
		//build master sim cntxt
		void buildMasterCntxt();
		//build structures holding all skeletons/worlds for multithreaded operation
		void buildSimCntxts();
		//initialize context launcher - manages multiple contexts per thread
		//void buildCntxtLaunchers();
		//initialize all goals to use master context pose as rest pose
		void initGoals();
		//refresh all goals at beginning of iteration step - to set variance bounds to be min/max results of last iteration of cpbp, if enabled
		void refreshAllGoals() { for (int i = 0; i < numGoals; ++i) { goals[i]->refreshGoal(); } }

		vector<double> lerpV4(vector<double>& a, double t, vector<double>& b) {
			vector<double> res(4);
			res[0] = (a[0] + t*(b[0] - a[0]));
			res[1] = (a[1] + t*(b[1] - a[1]));
			res[2] = (a[2] + t*(b[2] - a[2]));
			res[3] = (a[3] + t*(b[3] - a[3]));
			return res;
		}

	public: //variables
		dart::simulation::WorldPtr main_world;										//main cntxt world
		dart::dynamics::SkeletonPtr main_skel;

		std::shared_ptr<SimContext> masterSimCntxt;                                 //skel adapter for main skeleton being controlled
		dart::simulation::WorldPtr mFwdSimWorld;									//sim world used for single-world calculations

		//std::vector<dart::simulation::WorldPtr> mFwdSimWorlds;						//worlds for multithreaded calculation of forward physics sim

		std::vector<thread> sampleThreads;											//1 thread per context

		std::vector<std::shared_ptr<SimContext>> mFwdSimCntxts;						//simulation contexts (includes old skeleton adapters + PBP-specific info) for all forward particle/phisical sim worlds - provides quick access to important components of skeleton
		std::vector<std::shared_ptr<CntxtLauncher>> mCntxtLaunchers;				//list of context launchers spanning the entire list of fwd sim cntxts. each of these will be launched in a thread, and then in turn will iterate through its walkers.  allow multiple sim contxts per thread

		std::vector<std::shared_ptr<MotionGoal>> goals;								//goals for this motion that we use for cost calculation
		int numGoals;																//how many goals we will pursue simultaneously

		//Optimizer initialization params and helpers
		Eigen::VectorXd minControl, maxControl, 
			controlMean, controlPriorStd, controlDiffPriorStd, controlDiffDiffPriorStd,				//should have numCntrlDim members
			minAngles, maxAngles, angleRanges, angleStdScales, angleRngStdProd,												
			stateKernelStd;																			//should have nXStateDimensions					

		double simulationTime = 0,
				currEffortLvl;																		//needs to be between 0 and 1
		const int TORSO_IDX = 0, L_ARM_IDX = 1, R_ARM_IDX = 2, L_LEG_IDX = 3, R_LEG_IDX = 4;		//idx offset for "catagories" of bones, for control priors and min/max values

		ControlPBP pbp;																				//controlPBP instance
		std::shared_ptr<CPBPParams> cp;																//modfiable params and hyperparams

		int numTotalFrames,																			//# of frames since start
			numSimSteps,
			numStateDim,																			//# of dimensions/dofs describing reduced state used internally by ControlPBP
			numCntrlFlimDim,													
			numCntrl,
			numDofs;
		unsigned long numThdsHardware;														//hardware dictated # of threads supported without cntxt switching

		//UI
		std::shared_ptr<MyGuiHandler> UI;															//UI to enable modification of sim variables used by this class
		vector<vector<int>> UIObjIdx;																//the idx's of the UI objects corresponding to this class, idxed by type of obj and then list of idx's in that kind of obj's container

	};//CPBPHandler
}//namespace AaltoGames 
#endif  // APPS_PARTICLEBELIEFPROP_CPBPHandler_H_
