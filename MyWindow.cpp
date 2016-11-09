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

#include "MyWindow.h"
#include <time.h>
#include <Math.h>
//#include "MathUtils.h"
//#include "ControlPBP.h"
//#include "CPBPParser.h"

#include "dart/math/Helpers.h"
#include "dart/utils/SkelParser.h"
#include "dart/simulation/World.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/Joint.h"
#include "dart/dynamics/Marker.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/BoxShape.h"
#include "dart/gui/GLFuncs.h"
#include "dart/renderer/OpenGLRenderInterface.h"
#include "GL\glut.h"
#include <memory>

using namespace std;
using namespace Eigen;
using namespace cPBPropApp;

//==============================================================================
MyWindow::MyWindow()
	: SimWindow(), cpbp(nullptr), cp(nullptr), main_skel(nullptr), _wSM(1), UIList(0), mainCntxtCost(0), showUIBtnIdx(-1), genCoordIds(), initConfig(), initState(), activeMarkerCnt(0), clickTarget(0, 0, 0), selMrkrIDXs(), frcAppStr(),
	elapsedSimTime(0)
{
	mTrans[1] = 300.f;
	mEye[2] = 5;                //back up a little
	origTrackBallQ = Quaterniond(mTrackBall.getCurrQuat());
	_wSM.resize(numFlags, false);
	mDisplayTimeout = 1000.0 / 100.0;
}

//set initial state of flags - UI should be made first
void MyWindow::initFlags() {
	setFlag(debug, false, debug+1);
	setFlag(showSkels, false, showSkels+1);
	setFlag(showUI, true, showUI + 1);
	setFlag(showSimCnst, true, showSimCnst + 1);
}


void MyWindow::initWindow(int _w, int _h, const char* _name) {
	initSkelsAndWin(_w, _h);
	//overriding default behavior
	//from GlutWindow
	mWindows.push_back(this);

	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA | GLUT_MULTISAMPLE);
	glutInitWindowPosition(10, 10);//upper corner
	glutInitWindowSize(_w, _h);
	mWinIDs.push_back(glutCreateWindow(_name));

	glutDisplayFunc(refresh);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(keyEvent);
	glutSpecialFunc(specKeyEvent);
	glutMouseFunc(mouseClick);
	glutMotionFunc(mouseDrag);
	glutPassiveMotionFunc(mouseMove);

	delete mRI;
	mRI = new dart::renderer::OpenGLRenderInterface();
	mRI->initialize();

	//from Win3D
	int smaller = _w < _h ? _w : _h;
	mTrackBall.setTrackball(Eigen::Vector2d(_w*0.5, _h*0.5), smaller / 2.5);

}//initWindow


//load and initialize all skeletons - need numSamples # of skeletons/worlds for particle belief prop
void MyWindow::initSkelsAndWin( int _w, int _h) {
	mWinWidth = _w;
	mWinHeight = _h;
	//cp = std::allocate_shared<CPBPParams>(Eigen::aligned_allocator <CPBPParams>());
	cp = make_shared<CPBPParams>();
	Eigen::VectorXd tmpBuffer(100);		//need enough space for all possible initial configs
	std::string tmpStr = DART_DATA_PATH;
	CPBPParser::readCPBPConfig(DART_DATA_PATH"../apps/particleBeliefProp/BaseCPBPConfig.xml", tmpStr, cp, genCoordIds, tmpBuffer);

	initConfig = tmpBuffer.head(genCoordIds.size());			//default should be -0.2, -0.05, 0.15, -0.4, 0.25, 0.15, -0.4, 0.25, 0.0;
	cout << *cp << endl;
	Vector3d gravity(0.0, -9.81, 0.0);
	mWorld = dart::utils::SkelParser::readWorld(cp->skelFileName);
	mWorld->setGravity(gravity);
	mWorld->setTimeStep(cp->deltaT);
	main_skel = mWorld->getSkeleton(cp->idxInWorld);

	cpbp = std::allocate_shared<CPBPHandler>(Eigen::aligned_allocator <CPBPHandler>(), main_skel, mWorld, cp);
	cpbp->initCPBPHandler();				//build master cntxt, simulation cntxts, and context thread launchers
	//get num goals and numSubGoals vector from xml
	int numGoals = 1;
	vector<int> numSubGoals(1);
	
	//TODO get from xml
	numSubGoals[0] = 14;
	cpbp->buildGoals(numGoals, numSubGoals);						
	initSkels();
	cout << "Init UI pages " << endl;
	initUIPages();
	cpbp->setMasterPoseAsRestState(true);								//initializes all rest-state vars and assign to goals
	selMrkrIDXs.resize(cpbp->masterSimCntxt->markers.size());
	frcAppStr.resize(cpbp->masterSimCntxt->markers.size());
	for (int i = 0; i < selMrkrIDXs.size(); ++i) {
		selMrkrIDXs[i] = false;
		frcAppStr[i] = "";
	}
}

//build all UI pages
// MyWindow : debug output and interaction buttons
// CPBPHandler : hyperparameter tuning
// goals : weights, sds, means and params
//need 2 + # of goals UI pages (currently set 3)
void MyWindow::initUIPages() {
	//need to get all this stuff from XML ala Knowledge/Kinect proj TODO
	const int numPages = 3;
	UIList.resize(numPages);
	float stXVals[numPages] = { (mWinWidth * .75f), 0, (mWinWidth * .75f) };
	float stYVals[numPages] = { (mWinHeight * .85f), 0, 0 };

	for (int i = 0; i < numPages; ++i) {
		UIList[i] = std::make_shared<MyGuiHandler>(stXVals[i], stYVals[i], mWinWidth, mWinHeight);
		//UIList[i] = std::allocate_shared<MyGuiHandler>(Eigen::aligned_allocator <MyGuiHandler>(), stXVals[i], stYVals[i], mWinWidth, mWinHeight);
	}
	//this window UI is idx 0
	initUI();
	initFlags();		//initialize all flags for MyWindow instance
	cpbp->UI = UIList[1];
	cpbp->initUI();
	//send appropriate UI constructs to each goal
	for (int i = 0; i < cpbp->numGoals; ++i) {
		//set goal UI here
		cpbp->goals[i]->UI = UIList[i + 2];
		cpbp->goals[i]->initUI();
	}
}
//overrride default behavior - allow for timestep bigger than mDisplayTimeout, also keep framerate 
void MyWindow::displayTimer(int _val)  {//don't care about playback or any baking
	if (mSimulating) {
		elapsedSimTime += mWorld->getTimeStep() * 1000;							//sim time since beginning simulation TODO this is wrong
		timeStepping();

		//if (_wSM[debug]) {				cout << "timestep\n";}
		//the following allow for multiple fwd sim prediction steps per draw
		//double elapsedTSTime = 0,//fmod(elapsedSimTime, mDisplayTimeout),				//starting point of this frame, in elapsed milliseconds
		//	tsMult = mWorld->getTimeStep() * 1000;
		//while (mDisplayTimeout > elapsedTSTime) {
		//	elapsedTSTime += tsMult;												//sim time since this time step started
		//	elapsedSimTime += tsMult;												//sim time since beginning simulation
		//	timeStepping();
		//	mWorld->bake();															//sends results to mWorld->mRecording
		//	if (_wSM[debug]) {				cout << "timestep\n";}
		//}
		//double elapsedTSTime = 0;					
		//clock_t t;
		//t = clock();
		//while (mDisplayTimeout > elapsedTSTime) {
		//	//++iter;
		//	timeStepping();
		//	mWorld->bake();															//sends results to mWorld->mRecording
		//	t = (clock() - t);
		//	elapsedTSTime += 1000 * ((double)(t) / CLOCKS_PER_SEC);					//miliseconds since started this cycle
		//}
		glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
	}
	glutPostRedisplay();

}

//==============================================================================
void MyWindow::timeStepping(){
	if ((_wSM[selMrkr]) && (_wSM[dragMrkr]) ) {//if can drag-force, and are dragging then apply force
		applyMarkerPullForce();										//apply force at marker location(s) from dragging mouse
	}
	if (_wSM[beginCPBP] || (cpbp->masterSimCntxt->isTouchingGround())) {			//wait until touching the ground before iterating the cpbp code, or if we have started cpbp already	
	//if (_wSM[beginCPBP] || (cpbp->masterSimCntxt->COM(1) < .01)) {					//wait until almost touching the ground before iterating the cpbp code, or if we have started cpbp already	
		if (!_wSM[beginCPBP]) {
			static int cdown = 5;
			if (cdown < 0) {
				cpbp->setMasterPoseAsRestState(true);
				initState = cpbp->masterSimCntxt->currState;
				cpbp->initCPBP();														//initialize now so that we have rest pose on ground to set up as pose prior
				setFlag(beginCPBP, true);
				cpbp->iterateCPBP();
				if (_wSM[testSimDet]) { testSimCntxtDet(); return; }						//test to make sure that cntxt state and control is repeatable - needs to be called after at least 1 cycle of cpbp, so that 0 cntxt has a full set of controls and states
			}
			else {		cdown--;	}
		}
		else {																		//we just hit the ground, set master currState as the rest state and init cpbp - do only 1 time
			//cout << "Iterate CPBP : " << elapsedSimTime << "\n";
			cpbp->iterateCPBP();
			if (_wSM[testSimDet]) { testSimCntxtDet(); return; }						//test to make sure that cntxt state and control is repeatable - needs to be called after at least 1 cycle of cpbp, so that 0 cntxt has a full set of controls and states
		}
	}		
	mainCntxtCost = cpbp->stepMstrCntxtGetCost(_wSM[beginCPBP]);					//step simulation with derived control, and evaluate cost - mainCntxtCost is for display
}

//display results of debug tests to verify collision determinism
void MyWindow::displayDebugTestInfo(int idx, int cntrlIdx, int testIter) {
	if (testIter > 0) { cout << "Test run : " << testIter << " : \n"; }
	cpbp->mFwdSimCntxts[idx]->currState = cpbp->mFwdSimCntxts[0]->debugStepStartState[cntrlIdx];		//get state for all skeletons from cntxt 0's init debug state
	//cpbp->mFwdSimCntxts[idx]->displaceSkel(cpbp->mFwdSimCntxts[0]->skelLocInWorld, cpbp->mFwdSimCntxts[idx]->skelLocInWorld);
	cpbp->mFwdSimCntxts[idx]->updateSkelKinState();
	cout << "\tState of Context 0 Before : " << cpbp->mFwdSimCntxts[0]->buildStrFromEigenXd(cpbp->mFwdSimCntxts[0]->debugStepStartState[cntrlIdx]) << "\n";
	cout << "\tState of Context " << idx << " Before : " << cpbp->mFwdSimCntxts[idx]->buildStrFromEigenXd(cpbp->mFwdSimCntxts[idx]->currState) << "\n\n";
	//cpbp->mFwdSimCntxts[idx]->applyControl(cntrlIdx, cpbp->mFwdSimCntxts[0]->debugStepCntrl[cntrlIdx]);
	cpbp->mFwdSimCntxts[idx]->applyControl( cpbp->mFwdSimCntxts[0]->debugStepCntrl[cntrlIdx]);
	cpbp->mFwdSimCntxts[idx]->updateStateControl();
	cout << "\tControl Applied :" << cpbp->mFwdSimCntxts[0]->buildStrFromEigenXd(cpbp->mFwdSimCntxts[0]->debugStepCntrl[cntrlIdx]) << "\n\n";
	double nVal = (cpbp->mFwdSimCntxts[idx]->currState - cpbp->mFwdSimCntxts[0]->debugStepState[cntrlIdx]).norm();
	cout << (nVal != 0 ? "********** " : "\t") << " ---->Difference in States at State " << cntrlIdx << " : " << nVal;
	if (nVal != 0) { cout << " ********* "; }	cout << "\n\n";
	cout << "\tState of Context 0 After : " << cpbp->mFwdSimCntxts[0]->buildStrFromEigenXd(cpbp->mFwdSimCntxts[0]->debugStepState[cntrlIdx]) << "\n";
	cout << "\tState of Context " << idx<<" After : " << cpbp->mFwdSimCntxts[idx]->buildStrFromEigenXd(cpbp->mFwdSimCntxts[idx]->currState) << "\n";
	cout << "\tContacts at step " << cntrlIdx;
	if (testIter > 0 ) { cout<<" Test iteration : "<< testIter << "\n";}
	//for (int j = 0; j < cpbp->mFwdSimCntxts[0]->debugStepContacts[cntrlIdx].size(); ++j) {
	//	cout << "\t\tCntxt 0 contact : " << cntctToString(cpbp->mFwdSimCntxts[0]->debugStepContacts[cntrlIdx][j]) << "\n";
	//	if ((cpbp->mFwdSimCntxts[idx]->debugStepContacts.size() > cntrlIdx) && (cpbp->mFwdSimCntxts[idx]->debugStepContacts[cntrlIdx].size() > j)) {
	//		cout << "\t\tCntxt " << idx << " contact : " << cntctToString(cpbp->mFwdSimCntxts[idx]->debugStepContacts[cntrlIdx][j]) << "\n";
	//	}
	//	else { cout << "\t---->No " << j << "th contact for Cntxt : " << idx << " at step " << cntrlIdx << "\n"; }
	//}

	for (int j = 0; j < cpbp->mFwdSimCntxts[0]->debugStepContacts[cntrlIdx].size(); ++j) {
		cout << "\t\tCntxt 0 contact : " << cntctToString(cpbp->mFwdSimCntxts[0]->debugStepContacts[cntrlIdx][j]) << "\n";
	}
	cout << "\n";
	for (int j = 0; j < cpbp->mFwdSimCntxts[idx]->debugStepContacts[cntrlIdx].size(); ++j) {
		cout << "\t\tCntxt " << idx << " contact : " << cntctToString(cpbp->mFwdSimCntxts[idx]->debugStepContacts[cntrlIdx][j]) << "\n";
	}
	cout << "\n\n";
}

//take sim cntxt 0's list of controls, and initial state, apply controls and states to another cntxt, compare final states
void MyWindow::testSimCntxtDet() {
	int numTestCntxts_plus1 = cpbp->mFwdSimCntxts.size(); //set to 2 to compare only idx 0 to idx 1
	int testReps = 1;
	cout << "Test results : Apply context 0's start state and controls to context 1 " << testReps << " times repeatedly, compare contact results : \n ";
	for (int cIdx = 0; cIdx < cpbp->mFwdSimCntxts[0]->debugStepCntrl.size(); ++cIdx) {
		for (int t = 0; t < testReps; ++t) {
			displayDebugTestInfo(1, cIdx, t+1);
		}
	}
	cout << "Test Results : Each cntrl application from cntxt 0 to " << numTestCntxts_plus1 << " contexts : \n";
	//all skels inited to cntxt 0's state
	for (int i = 1; i < numTestCntxts_plus1; ++i) {//using idx 0 as source of control and start state
		cout << "Cntxt : " << i << ":\n";
		for (int cIdx = 0; cIdx < cpbp->mFwdSimCntxts[0]->debugStepCntrl.size(); ++cIdx) {
			displayDebugTestInfo(i, cIdx, 0);
		}
	}
	cout << "Test Results : Total : " << endl;
	vector<double> resStateVals(1);
	//all skels have been moved by here. now compare states to final state of cntxt 0
	for (int i = 1; i < numTestCntxts_plus1; ++i) {//using idx 0 as source of control and start state
		cpbp->mFwdSimCntxts[i]->getDartSkelState(cpbp->mFwdSimCntxts[i]->s, Eigen::Vector3d(0,0,0));
		double nVal = (cpbp->mFwdSimCntxts[i]->currState - cpbp->mFwdSimCntxts[0]->debugStepState[cpbp->mFwdSimCntxts[0]->debugStepState.size() - 1]).norm();
		cout << "i : " << i << " : norm of diff state vector " << nVal << "\n";
		resStateVals.push_back(nVal);
	}
}

//==============================================================================

void MyWindow::draw() {//override base draw, so can render UI
	SimWindow::draw();			//base class version first
	//if (_wSM[debug]) {	cout << "draw\n";	}
	draw2dUI();					//draw all UI components
}
//==============================================================================
void MyWindow::drawSkels() {
	glEnable(GL_LIGHTING);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	
	drawCOM(cpbp->masterSimCntxt->COM, cpbp->masterSimCntxt->COP, cpbp->masterSimCntxt->COMLinVel, cpbp->masterSimCntxt->COPCalcPt(1));
	////debug only
	//auto simCtxt = cpbp->masterSimCntxt;
	//Eigen::Vector3d
	//	leftFootLoc = simCtxt->nodes[simCtxt->cp->lftHeelRBIdx]->getCOM(), rightFootLoc = simCtxt->nodes[simCtxt->cp->rtHeelRBIdx]->getCOM(),
	//	leftToeLoc = simCtxt->nodes[simCtxt->cp->lftToeRBIdx]->getCOM(), rightToeLoc = simCtxt->nodes[simCtxt->cp->rtToeRBIdx]->getCOM(),
	//	leftFtBallLoc = leftFootLoc,//.5*(leftFootLoc + leftToeLoc), 
	//	rightFtBallLoc = rightFootLoc;// .5*(rightFootLoc + rightToeLoc);
	//leftFtBallLoc(1) = cpbp->masterSimCntxt->COPCalcPt(1);						rightFtBallLoc(1) = cpbp->masterSimCntxt->COPCalcPt(1);
	//glPushMatrix();
	//	glTranslated(0,0,2);//move away so can see feet arrangement
	//	drawCOM(cpbp->masterSimCntxt->COM, cpbp->masterSimCntxt->COP, cpbp->masterSimCntxt->COMLinVel, cpbp->masterSimCntxt->COPCalcPt(1));
	//	drawCOM(leftFtBallLoc, cpbp->masterSimCntxt->COP, cpbp->masterSimCntxt->COMLinVel, cpbp->masterSimCntxt->COPCalcPt(1));
	//	drawCOM(rightFtBallLoc, cpbp->masterSimCntxt->COP, cpbp->masterSimCntxt->COMLinVel, cpbp->masterSimCntxt->COPCalcPt(1));


	//glPopMatrix();
	//end debug only
	drawAxes(Vector3d(0, 1, 0), .2f, false);
	if (_wSM[showSkels]) { drawFwdWalkers(); }
	if (_wSM[beginCPBP]) { drawWalkerTraj(); }					//only draw trajectories if they exist (i.e. if cpbp has started)
	SimWindow::drawSkels();
	if (_wSM[selMrkr]) { main_skel->drawMarkers(mRI); }			//draw markers on skel
}

void MyWindow::draw2dUI() {
	glPushMatrix();
	enable2dUIMode(true);
	if (_wSM[showUI]) {		for (int i = 0; i < UIList.size(); ++i) {	UIList[i]->drawObjs();}	}//draw all UI components
	else {								//draw only main window buttons
		UIList[0]->drawObjs(); //UIList[0]->drawObj(showUIBtnIdx);
	}
	drawCostAndResamp();
	if (_wSM[debug]) {
		for (int i = 0; i < UIList.size(); ++i) { UIList[i]->drawBoundBox(); } //draw UI bound box
		stringstream ss; ss.str(""); ss << "X:" << this->mMouseX << " Y:" << this->mMouseY;	drawTextAtLocation(10, .994*mWinHeight, ss.str(), 1.1, .08);//mouse location
	}
	if (_wSM[selMrkr]) {
		float stX = 5, stY = 1.01*UIList[1]->height + 12, incrY = 12;		//UIList[1] is CPBPhandler, we want to put these right underneath
		for (int i = 0; i < selMrkrIDXs.size(); ++i) {
			if (selMrkrIDXs[i]) {
				stY += incrY;
				drawTextAtLocation(stX, stY, frcAppStr[i], 1.1, .08);
			}
		}
	}
	enable2dUIMode(false);
	glPopMatrix();
}//		

//draw costs and resampling idx pruning paths on screen
void MyWindow::drawCostAndResamp() {
	float stX = 5, stY = 1.01*UIList[1]->height, incrY = 12;		//UIList[1] is CPBPhandler, we want to put these right underneath
	stringstream ss; 
	ss.str(""); ss << "Main Context Cost : " << buildStrFromFloat(mainCntxtCost, "%.4f");	drawTextAtLocation(stX, stY, ss.str(), 1.1, .08);
	stY += incrY;
	ss.str(""); ss << "Main Context Jerk Cost :" << buildStrFromFloat(cpbp->masterSimCntxt->jerkCost, "%.4f");	drawTextAtLocation(stX, stY, ss.str(), 1.1, .08);
	if ((_wSM[debug]) &&  (!_wSM[selMrkr])) {
		if (_wSM[showSimCnst]) {
			for (int i = 0; i < cpbp->mFwdSimCntxts.size(); ++i) {
				ss.str("");		stY += incrY;
				ss << i << ":" << cpbp->mFwdSimCntxts[i]->debugCstOut();
				drawTextAtLocation(stX, stY, ss.str(), 1.1, .06);//mouse location
			}//debugOut() 
		}
		else {
			for (int i = 0; i < cpbp->mFwdSimCntxts.size(); ++i) {
				ss.str("");		stY += incrY;
				ss << i << ":" << cpbp->mFwdSimCntxts[i]->debugPidxOut();
				drawTextAtLocation(stX, stY, ss.str(), 1.1, .06);//mouse location
			}//debugOut() 	
		}
	}
}//drawCostAndResamp

//build and draw full trajectories of root for each predictive walker to horizon, to display on main walker for debug purposes
//have best trajectory is red, back prop traj is green
void MyWindow::drawWalkerTraj() {
	glPushMatrix();
	glTranslatef(0, 0, 1);

	int numTrajSteps = cpbp->cp->numFwdTimeSteps, numThds = cpbp->mFwdSimCntxts.size();
	Vector3d st(0,0,0), end(0,0,0);
	for (int i = 2; i < numThds; ++i) {
		//for each sim context, draw trajectory of pelvis
		//end = (cpbp->mFwdSimCntxts[i]->debugStepState[0]).segment<3>(3);			//3 components of pelvis location
		for (int j = 1; j < numTrajSteps; ++j) {
			st = (cpbp->mFwdSimCntxts[i]->debugStepStartState[j]).segment<3>(3);
			end = (cpbp->mFwdSimCntxts[i]->debugStepState[j]).segment<3>(3);
			drawVecLine(.1f, .1f, .1f, .1f, st, end - st);
		}
	}
	//draw pbp traj 2nd-last to overlay others debugStepStartState
	//end = (cpbp->mFwdSimCntxts[1]->debugStepState[0]).segment<3>(3);			//3 components of pelvis location
	for (int j = 1; j < numTrajSteps; ++j) {
		st = (cpbp->mFwdSimCntxts[1]->debugStepStartState[j]).segment<3>(3);
		end = (cpbp->mFwdSimCntxts[1]->debugStepState[j]).segment<3>(3);
		drawVecLine(((1.0* numTrajSteps) - j) / (1.0* numTrajSteps), 1, ((1.0* numTrajSteps) - j) / (1.0* numTrajSteps), .7f, st, end - st);
	}
	//draw best traj last to overlay others
	end = (cpbp->mFwdSimCntxts[0]->debugStepState[0]).segment<3>(3);			//3 components of pelvis location
	for (int j = 1; j < numTrajSteps; ++j) {
		st = (cpbp->mFwdSimCntxts[0]->debugStepStartState[j]).segment<3>(3);;
		end = (cpbp->mFwdSimCntxts[0]->debugStepState[j]).segment<3>(3);
		drawVecLine(1, ((1.0* numTrajSteps) - j) / (1.0* numTrajSteps), ((1.0* numTrajSteps) - j) / (1.0* numTrajSteps), .7f, st, end - st);
	}
	glPopMatrix();
}

void MyWindow::drawFwdWalkers() {
	glPushMatrix();
	glTranslatef(-3, 0, -1);
	int prevSmplIdx = cpbp->mFwdSimCntxts[0]->prevSampleIDX = -1 ? 0 : cpbp->mFwdSimCntxts[0]->prevSampleIDX;
	int modVal = (int)(sqrt(cpbp->mFwdSimCntxts.size()) + 2);
	for (int i = 0; i < cpbp->mFwdSimCntxts.size(); ++i) {
		glPushMatrix();
		glTranslatef((2 - (modVal / 2) + (i%modVal)) , 0 , (-1 * i / modVal) );
		glTranslatef(-cpbp->mFwdSimCntxts[i]->skelLocInWorld(0), -cpbp->mFwdSimCntxts[i]->skelLocInWorld(1), -cpbp->mFwdSimCntxts[i]->skelLocInWorld(2));
		//glTranslatef((2 - (modVal / 2) + (i%modVal)) - cpbp->mFwdSimCntxts[i]->skelLocInWorld(0), -cpbp->mFwdSimCntxts[i]->skelLocInWorld(1), (-1 * i / modVal) - cpbp->mFwdSimCntxts[i]->skelLocInWorld(2));
		cpbp->mFwdSimCntxts[i]->s->draw(mRI, Eigen::Vector4d(1, 0, (prevSmplIdx == i ? 1 : 0), .2), false);
		drawCOM(cpbp->mFwdSimCntxts[i]->COM, cpbp->mFwdSimCntxts[i]->COP, cpbp->mFwdSimCntxts[i]->COMLinVel, cpbp->mFwdSimCntxts[i]->COPCalcPt(1));
		//stringstream ss;
		//ss << i << ":" << cpbp->mFwdSimCntxts[i]->name << ":  COM: " << cpbp->mFwdSimCntxts[i]->COM(0) << ", " << cpbp->mFwdSimCntxts[i]->COM(1) << ", " << cpbp->mFwdSimCntxts[i]->COM(2) << " : COP : " << cpbp->mFwdSimCntxts[i]->COP(0) << ", " << cpbp->mFwdSimCntxts[i]->COP(1) << ", " << cpbp->mFwdSimCntxts[i]->COP(2);
		//string res = ss.str();
		glPopMatrix();
	}
	glPopMatrix();
}

void MyWindow::getCurColor() { glGetFloatv(GL_CURRENT_COLOR, cc); }
void MyWindow::setOldColor() { glColor4f(cc[0], cc[1], cc[2], cc[3]); }

//draw com, cop and other quantities for this skeleton
void MyWindow::drawCOM(const Vector3d& com, const Vector3d& cop, const Vector3d& comVel, double gndLocY) {

	GLUquadricObj *c;
	c = gluNewQuadric();
	gluQuadricDrawStyle(c, GLU_FILL);
	gluQuadricNormals(c, GLU_SMOOTH);
	if (_wSM[selMrkr]) { drawSelMrkrDragForce(c); }
	drawVecBall(c, .02, 8, 8, .2, 0, .2, 1,com);
	drawVecBall(c, .02, 8, 8, 0, 1, 1, 1, cop);
	drawVecBall(c, .02, 8, 8, 1, 0, 1, 1, Eigen::Vector3d(com(0), gndLocY, com(2)));
	drawVecLine(.6f, 0, 1, 1, com, Vector3d(0, -com(1), 0));//height of com
	drawVecLine(0, 0, 0, 1, com, Vector3d((cop - com)));//line from cop to com
	drawVecLine(1, 0, 0, 1, com, comVel);//com vel

	gluDeleteQuadric(c);

}//drawcom

//draw the current drag force and the affected markers
void MyWindow::drawSelMrkrDragForce(GLUquadricObj *c) {
	Eigen::Vector3d totFrc(0,0,0);
	for (int i = 0; i < selMrkrIDXs.size(); ++i) {
		if (selMrkrIDXs[i]) {
			Eigen::Vector3d mLoc = cpbp->masterSimCntxt->markers[i]->getWorldPosition();
			drawVecBall(c, .02, 8, 8, 1, 0, 0, 1, mLoc); //if showing markers, draw selected markers in different color 
			if (_wSM[dragMrkr]) {
				drawFrcVecLine(0, 0, 0, 1, mLoc, (clickTarget - mLoc));//vector from marker to drag location
			}
			totFrc += (clickTarget - mLoc);
		}
	}
	if (_wSM[dragMrkr]) {
		drawVecBall(c, .02, 8, 8, 1, 0, 0, 1, clickTarget); //if showing markers, draw target ball
	}
}//drawDragForce
//draw a line from st in mv dir
void MyWindow::drawVecLine(GLfloat cr, GLfloat cg, GLfloat cb, GLfloat ca, const Vector3d& st, const Vector3d& mv) {
	glPushMatrix();
	glColor4f(cr, cg, cb, ca);
	glTranslated(st(0), st(1), st(2));
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(mv(0), mv(1), mv(2));           
	glEnd();
	glPopMatrix();
}
//draw a force vector arrow from st in mv dir
void MyWindow::drawFrcVecLine(GLfloat cr, GLfloat cg, GLfloat cb, GLfloat ca, const Vector3d& st, const Vector3d& mv) {
	glPushMatrix();
	glColor4f(cr, cg, cb, ca);
	Eigen::Vector3d stOff(.1*mv);
	Eigen::Vector3d endOff(.9*mv);
	Eigen::Vector3d arrowHead(.8*mv);
	Eigen::Vector3d prpVec(0, 0, 0);
	prpVec = ((mv.cross(st)).cross(mv)).normalized();
	prpVec *= (.1*mv.norm());


	glTranslated(st(0), st(1), st(2));
	glBegin(GL_LINES);
	glVertex3d(stOff(0), stOff(1), stOff(2));
	glVertex3d(endOff(0), endOff(1), endOff(2));

	glVertex3d(endOff(0), endOff(1), endOff(2));
	glVertex3d(arrowHead(0) + prpVec(0), arrowHead(1) + prpVec(1), arrowHead(2) + prpVec(2));
	glVertex3d(arrowHead(0) + prpVec(0), arrowHead(1) + prpVec(1), arrowHead(2) + prpVec(2));
	glVertex3d(arrowHead(0), arrowHead(1), arrowHead(2));

	glVertex3d(endOff(0), endOff(1), endOff(2));
	glVertex3d(arrowHead(0) - prpVec(0), arrowHead(1) - prpVec(1), arrowHead(2) - prpVec(2));
	glVertex3d(arrowHead(0) - prpVec(0), arrowHead(1) - prpVec(1), arrowHead(2) - prpVec(2));
	glVertex3d(arrowHead(0), arrowHead(1), arrowHead(2));
	glEnd();
	glRotated(90, endOff(0), endOff(1), endOff(2));
	glBegin(GL_LINES);
	glVertex3d(endOff(0), endOff(1), endOff(2));
	glVertex3d(arrowHead(0) + prpVec(0), arrowHead(1) + prpVec(1), arrowHead(2) + prpVec(2));
	glVertex3d(arrowHead(0) + prpVec(0), arrowHead(1) + prpVec(1), arrowHead(2) + prpVec(2));
	glVertex3d(arrowHead(0), arrowHead(1), arrowHead(2));

	glVertex3d(endOff(0), endOff(1), endOff(2));
	glVertex3d(arrowHead(0) - prpVec(0), arrowHead(1) - prpVec(1), arrowHead(2) - prpVec(2));
	glVertex3d(arrowHead(0) - prpVec(0), arrowHead(1) - prpVec(1), arrowHead(2) - prpVec(2));
	glVertex3d(arrowHead(0), arrowHead(1), arrowHead(2));
	glEnd();
	glPopMatrix();
}
void MyWindow::drawVecBall(GLUquadricObj *c, GLdouble rad, GLint sl, GLint st, GLfloat cr, GLfloat cg, GLfloat cb, GLfloat ca, const Vector3d& coord) {
	glPushMatrix();
	glColor4f(cr, cg, cb, ca);
	glTranslatef(coord(0), coord(1), coord(2));
	gluSphere(c, rad, sl, st);
	glPopMatrix();
}
//toggle 2d ui mode or 3d object mode in drawing
void MyWindow::enable2dUIMode(bool on) {
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	if (on) {
		glDisable(GL_LIGHTING | GL_DEPTH_TEST);
		glDepthMask(0);
		glOrtho(0, mWinWidth, mWinHeight, 0, -1, 1);
	}
	else {
		glEnable(GL_DEPTH_TEST | GL_LIGHTING);
		glDepthMask(1);
		gluPerspective(mPersp,
					   static_cast<double>(mWinWidth) / static_cast<double>(mWinHeight),
					   0.1, 10.0);
	}
	glViewport(0, 0, mWinWidth, mWinHeight);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}//set2dUIMode


void MyWindow::drawTextAtLocation(float x, float y, string text, float lineW, float scl) {
	glPushMatrix();
	glTranslatef(x, y, 0);
	glColor4f(0, 0, 0, 1);
	glLineWidth(lineW);			//default to 1.25f ?
	int lblLen = text.length();
	int chrStrkLen = 0;                                                         //length of glut stroke chars in "units" - use to dtermine size scaling
	glScalef(scl, -scl, 1);
	for (int i = 0; i < lblLen; ++i) { glutStrokeCharacter(GLUT_STROKE_ROMAN, text[i]); }
	glPopMatrix();

}//drawText

void MyWindow::drawAxes(const Vector3d& axesLoc, float len, bool altColor) {
	float col = (altColor ? .5f : 0);
	glPushMatrix();
	glTranslatef(axesLoc(0), axesLoc(1), axesLoc(2));
	glPushMatrix();
	glColor4f(1.0f, col, 0, 1.0f);
	glBegin(GL_LINES);                glVertex3f(0, 0, 0);                glVertex3f(len, 0, 0);            glEnd();
	glColor4f(0, 1.0f, col, 1.0f);
	glBegin(GL_LINES);                glVertex3f(0, 0, 0);                glVertex3f(0, len, 0);            glEnd();
	glColor4f(col, 0, 1.0f, 1.0f);
	glBegin(GL_LINES);                glVertex3f(0, 0, 0);                glVertex3f(0, 0, len);            glEnd();
	glPopMatrix();
	glPopMatrix();
}


void MyWindow::drawJointAxis(dart::dynamics::BodyNode* node) {
	glPushMatrix();
	node->getParentJoint()->applyGLTransform(mRI);      //gives location but not orientation
	drawAxes(Vector3d(0, 0, 0), .1f, false);
	for (unsigned int i = 0; i < node->getNumChildBodyNodes(); i++) {
		drawJointAxis(node->getChildBodyNode(i));
	}
	glPopMatrix();
}

void MyWindow::drawJointAxes() {//Vector3d nodeY = node->getWorldTransform().linear().col(1); 
	drawJointAxis(main_skel->getRootBodyNode());
}

//==============================================================================
//check each UI page to see if it contains an element that has been clicked or dragged and needs processing
int MyWindow::checkPagesForClick(int _button, int _eventType, int& _x, int& _y) {
	int depth = 0;													//NOT USED - only used by guiHandler for kinect input
	if (_wSM[showUI]) {//if showing UI
		for (int i = 0; i < UIList.size(); ++i) {		//find which UI page holds the element that has been clicked/modified
			if (UIList[i]->checkUIElements(_x, _y, depth, true, _button, _eventType)) { glutPostRedisplay(); return i; }
		}
	}
	else {//if not showing UI, check if click in this window's UI only
		if (UIList[0]->checkUIElements(_x, _y, depth, true, _button, _eventType)) { glutPostRedisplay(); return 0; }//if (UIList[0]->checkUIElement(showUIBtnIdx, _x, _y, depth, true, _button, _eventType)) { return 0; }
	}
	return -1;
}
//map click coordinates to closest marker on skeleton
int MyWindow::coordsToMarker(int _x, int _y) {
	GLuint selectBuf[512];
	GLint hits;
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);
	glSelectBuffer(512, selectBuf);
	glRenderMode(GL_SELECT);						//no frags are produced, instead list of primitives that would have been drawn is returned in glSelectBuffer selectBuf
	glInitNames();

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPickMatrix(_x, mWinHeight - _y, 2.0, 2.0, viewport);
	gluPerspective(mPersp, (double)mWinWidth / (double)mWinHeight, 0.1, 10.0);
	gluLookAt(mEye[0], mEye[1], mEye[2], 0.0, 0.0, -1.0, mUp[0], mUp[1], mUp[2]);
	//gluLookAt(mEye[0], mEye[1], mEye[2], cpbp->masterSimCntxt->COM(0), cpbp->masterSimCntxt->COM(1), cpbp->masterSimCntxt->COM(2), mUp[0], mUp[1], mUp[2]);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	mTrackBall.applyGLRotation();
	glScalef(mZoom, mZoom, mZoom);
	glTranslatef(mTrans[0] * 0.001, mTrans[1] * 0.001, mTrans[2] * 0.001);
	main_skel->drawMarkers(mRI);

	hits = glRenderMode(GL_RENDER);				//return to render, return # of hits in selectBuf

	if (hits) {		return processHits(hits, selectBuf);	}
	return -1;
}
//determine which marker was clicked
int MyWindow::processHits(GLint _hits, GLuint _buffer[]) {
	GLfloat minz = 9999999;
	GLfloat tempz;
	GLuint *ptr;
	ptr = _buffer;
	int selectedMarker;

	for (int i = 0; i < _hits; i++) {
		ptr++;
		tempz = (float)*ptr / 0x7fffffff;
		if (tempz < minz) {
			minz = tempz;
			selectedMarker = *(ptr + 2);
		}
		ptr += 3;
	}
	return selectedMarker;
}
//handles only 1 target currently - can be modified easily to handle multiples by maintaining a click target list idx'ed by activeMarkerIDX
void MyWindow::createConstraint(int idx) { clickTarget = cpbp->masterSimCntxt->markers[idx]->getWorldPosition(); selMrkrIDXs[idx] = true; frcAppStr[idx] = ""; activeMarkerCnt++; }
void MyWindow::modifyConstraint(const Eigen::Vector3d& _deltaP) {	clickTarget += _deltaP;}
void MyWindow::removeConstraint(int idx) { clickTarget = cpbp->masterSimCntxt->markers[idx]->getWorldPosition();	selMrkrIDXs[idx] = false; frcAppStr[idx] = ""; activeMarkerCnt--; }
void MyWindow::clearAllConstraints() {
	clickTarget = cpbp->masterSimCntxt->markers[0]->getWorldPosition();
	activeMarkerCnt = 0;
	for (int i = 0; i < selMrkrIDXs.size(); ++i) { 
		selMrkrIDXs[i] = false; 
		frcAppStr[i] = "";
	}
}
//create force from all active markers to clickTarget
void MyWindow::applyMarkerPullForce() {
	Eigen::Vector3d applyForce(0, 0, 0);
	Eigen::Vector3d glbLoc, lclOff;// , newClickTarget;
	dart::dynamics::BodyNode* pnode;
	stringstream ss;

	//newClickTarget = cpbp->masterSimCntxt->markers[0]->getWorldPosition();
	float frcMult = 1000.0f;					//apply force scaling factor
	for (int i = 0; i < selMrkrIDXs.size(); ++i) {
		if (selMrkrIDXs[i]) {
			pnode = cpbp->masterSimCntxt->markers[i]->getBodyNode();
			glbLoc = cpbp->masterSimCntxt->markers[i]->getWorldPosition();
			//newClickTarget = glbLoc;
			lclOff = cpbp->masterSimCntxt->markers[i]->getLocalPosition();
			applyForce = frcMult * (clickTarget - glbLoc);
			pnode->addExtForce(applyForce, lclOff);
			ss << "Force applied to node : " << pnode->getName() << " at offset location : " << buildStrFromEigen3d(lclOff) << " : " << buildStrFromEigen3d(applyForce) <<"\n";
			frcAppStr[i] = ss.str();
			ss.str("");
			cout << frcAppStr[i];
		}//if sel
	}//for each marker
	_wSM[altPressed] = false;
	//clickTarget = newClickTarget;			//after apply force, get rid of clickTarget displacement
}//applyMarkerPullForce

Vector3d MyWindow::reverseProjection(double _x, double _y) {
	Matrix3d rot = mTrackBall.getRotationMatrix();
	Vector3d deltaP = rot.transpose() * Vector3d(_x, -_y, 0.0) / 1000.0;
	deltaP /= mZoom;
	return deltaP;
}

//mouse button is GLUT_LEFT_BUTTON,  GLUT_MIDDLE_BUTTON, or  GLUT_RIGHT_BUTTON
//state is  _state == GLUT_DOWN = 0 for click and  _state == GLUT_UP  = 1 for release
void MyWindow::click(int _button, int _state, int _x, int _y) {
	_wSM[altPressed] = false;
	int depth = 0,
		clickIDX,
		UIidx = checkPagesForClick( _button, 1 - _state,  _x,  _y);
	if (UIidx == -1) {//no click in any UI component or click released
		mMouseDown = !mMouseDown;
		int mask = glutGetModifiers();
		if (mMouseDown) {
			if (_button == GLUT_LEFT_BUTTON) {
				if (mask == GLUT_ACTIVE_SHIFT) {	mZooming = true;}
				else if (mask == GLUT_ACTIVE_ALT) {					//if marker select is enabled
					_wSM[altPressed] = true;
					if (_wSM[selMrkr]) {
						clickIDX = coordsToMarker(_x, _y);
						if (clickIDX != -1) { 
							(selMrkrIDXs[clickIDX]) ? 
								removeConstraint(clickIDX) : 
								createConstraint(clickIDX); 
						}
					}
				}
				else {		
					mRotate = true;		
					mTrackBall.startBall(_x, mWinHeight - _y);						
				}
			}
			else if (_button == GLUT_RIGHT_BUTTON || _button == GLUT_MIDDLE_BUTTON) {
				mTranslate = true;	
			}
			else if (_button == 3 && _state == GLUT_DOWN) {		mZoom += 0.1;		}
			else if (_button == 4 && _state == GLUT_DOWN) {		mZoom -= 0.1;		}
			mMouseX = _x;
			mMouseY = _y;
		}
		else {
			mTranslate = false;
			mRotate = false;
			mZooming = false;
			if ((_wSM[selMrkr]) && (_wSM[dragMrkr]) && (mask == GLUT_ACTIVE_ALT)) {
				for (int i = 0; i < selMrkrIDXs.size(); ++i) {
					if (selMrkrIDXs[i]) {
						clickTarget = cpbp->masterSimCntxt->markers[i]->getWorldPosition();
						break;
					}
				}
				//applyMarkerPullForce();										//apply force at marker location(s) from dragging mouse
			}	
		}
		glutPostRedisplay();
	}
	else {
		//these checks also handle the click action - would be nice to fold refs to owning pages here so that we don't have to check every page's every object every time - we have idx from checkPagesForClick
		if (!handleBtnClick()) {			//check if myWindow's UI owns the object clicked
			if (!cpbp->handleBtnClick()) {	//check if cpbp's UI owns the oject clicekd
				int i = 0; bool found = false;
				while ((i < cpbp->numGoals) && (!found)) {	//check each of the possibly multiple goals if they own the button clicked
					found = cpbp->goals[i]->handleBtnClick();
					i++;
				}
			}
		}
		mMouseX = _x;
		mMouseY = _y;
	}
	_wSM[dragMrkr] = false;			//always set dragging to false upon release
}//click
//mouse move while clicked
void MyWindow::drag(int _x, int _y) {
	int depth = 0,
		UIidx = checkPagesForClick(0, 2, _x, _y);
	if (UIidx == -1) {											//no drag click in any UI component
		double deltaX = _x - mMouseX;
		double deltaY = _y - mMouseY;
		mMouseX = _x;
		mMouseY = _y;
		if ((_wSM[selMrkr]) && (_wSM[altPressed]) && (activeMarkerCnt > 0)) {
			Vector3d deltaP = reverseProjection(deltaX, deltaY);
			modifyConstraint(deltaP);
			_wSM[dragMrkr] = true;
		}
		if (mRotate) {
			if (deltaX != 0 || deltaY != 0) { mTrackBall.updateBall(_x, mWinHeight - _y); }
		}
		if (mTranslate) {
			Matrix3d rot = mTrackBall.getRotationMatrix();
			mTrans += rot.transpose() * Vector3d(deltaX, -deltaY, 0.0);
		}
		if (mZooming) {			mZoom += deltaY * 0.01;		}
		glutPostRedisplay();
	}
	else {
		this->mMouseX = _x;
		this->mMouseY = _y;
	}
}//drag

//mouse move while not clicked - only for debug
void MyWindow::move(int _x, int _y) {
	this->mMouseX = _x;
	this->mMouseY = _y;
}//move

void MyWindow::keyboard(unsigned char _key, int _x, int _y){
	cout << "key pressed : "<< _key << "\n";
	int mask = glutGetModifiers();
	cout << "modifiers mask : " << mask << "\n";
	bool callDefault = false;

	switch (_key)  {
		case ' ':	{ toggleSimState(); break; }					// use space key to play or stop the motion
		case 's': {break; }
		//case 'x': {dispVelOrDir = !dispVelOrDir; cout << "Now displaying " << (dispVelOrDir ? "Mtr Velocity" : "Position") << endl; dispCtr = 0; break; }
		case 'P':
		case 'p': { mPlay = !mPlay;if (mPlay) {	  mSimulating = false;  }  break; }								// playBack
		//case '[': { if (!mSimulating) { mPlayFrame--;  if (mPlayFrame < 0)  mPlayFrame = 0;  glutPostRedisplay(); }  break; }										// step backward
		//case ']': { if (!mSimulating) { mPlayFrame++;  if (mPlayFrame >= mWorld->getRecording()->getNumFrames())  mPlayFrame = 0; glutPostRedisplay(); }  break; }	// step forward
		case '[': { if (!mSimulating) { mPlayFrame--;  if (mPlayFrame < 0)  mPlayFrame = 0;   }  break; }										// step backward
		case ']': { if (!mSimulating) { mPlayFrame++;  if (mPlayFrame >= mWorld->getRecording()->getNumFrames())  mPlayFrame = 0; }  break; }	// step forward
		case 'V':
		case 'v': { mShowMarkers = !mShowMarkers;  break; }																											// show or hide markers
		default: {Win3D::keyboard(_key, _x, _y); callDefault = true; }
	}
	glutPostRedisplay();
}

//==============================================================================

//custom handling for sim state only because its also used in simwindow
void MyWindow::setSimState(bool newState) {
	mSimulating = newState;
	if (mSimulating) {
		mPlay = false;
		glutTimerFunc(mDisplayTimeout, refreshTimer, 0);
		UIList[0]->MyPgUIBtn[0]->label = "Stop Sim";
	}
	else {
		mPlayFrame = mWorld->getRecording()->getNumFrames();
		UIList[0]->MyPgUIBtn[0]->label = "Start Sim";
	}
	glutPostRedisplay();

}
//_btnIdx == 
void MyWindow::setFlag(int idx, bool _val, int _btnIdx) {
	_wSM[idx] = _val;
	switch (idx) {
		case beginCPBP: { break; }
		case debug: { UIList[0]->MyPgUIBtn[_btnIdx]->label = (_val ? "Debug Off" : "Debug On"); break; }
		case showSkels: { UIList[0]->MyPgUIBtn[_btnIdx]->label = (_val ? "Hide Walkders" : "Show Walkers"); break; }
		case showUI: { UIList[0]->MyPgUIBtn[_btnIdx]->label = (_val ? "Hide UI" : "Show UI"); break; }
		case showSimCnst: {UIList[0]->MyPgUIBtn[_btnIdx]->label = (_val ? "Show Pidx" : "Show Cost"); break; }
		case testSimDet: {UIList[0]->MyPgUIBtn[_btnIdx]->label = (_val ? "Stop Test" : "Test Det"); break; }
		case selMrkr: {UIList[0]->MyPgUIBtn[_btnIdx]->label = (_val ? "Hide Markers" : "Grab Markers"); clearAllConstraints(); break; }
	}
}//setFlag

//build the UI construct to modify the values of this class
void MyWindow::buildUI() {
	//UIObjIdx
	//TODO : read this in from XML?
	const int numBtns = 8;
	string buttonNames[] = { string("Start Sim"), string("Reset Sim"), string("Debug On"), string("Show Walkers"), string("Show UI"), string("Show Pidx"), string("Test Det"), string("Grab Markers") };		//idx 4 is show UI  - need to always display this button
	//UIList[0] is UI for MyWindow
	int numObjs = numBtns;									//all buttons + caption for each walker (to hold final costs if wishing to display them) ->CPBPHandler::numSamples
	UIList[0]->numObjs += numObjs;							//# of ui objects
	vector<int> objType;									//what type each object is : 0 : button, 1 : slider, 2 : caption, 3 : textbox
	vector<string> objLabels;								//object labels
	vector<vector<int>> objXY_WH;
	vector<vector<float>> objClr;

	objType.resize(numObjs);								//what type each object is : 0 : button, 1 : slider, 2 : caption, 3 : textbox
	objXY_WH.resize(numObjs);
	objLabels.resize(numObjs);
	objClr.resize(numObjs);
	int idx = 0;
	stringstream ss;
	float initY = 10, sliderLen = 20, col2 = UIList[0]->sldrBarLen + sliderLen + sliderLen,
		capWide = 30, btnHigh = 25;
	int numCols = 4;										//# of cols of buttons
	int numRows = (int)((numBtns - 1) / (numBtns < numCols ? numBtns : numCols)) + 1;				//# of rows, with numCols buttons per row
	UIList[0]->width = UIList[0]->winX - UIList[0]->stX - 1;			//extends to edge of screen

	float perBtnWidth = UIList[0]->width / numCols,
		btnWide = .85* perBtnWidth, 
		btnBuff = numCols > 0 ? (UIList[0]->width - (btnWide*numCols)) / (numCols - 1) : 0;

	////main walker - move to below buttons, add other walkers' costs
	//ss.str("");		ss << "Main Walker Cost : ";
	//vector<float> v2 = { initY, (btnHigh + initY), capWide, capWide, .25f, .25f, .25f, 1 };
	//UIList[0]->initUIObj(idx++, ss.str(), 2, v2, objType, objLabels, objXY_WH, objClr);

	for (int row = 0; row < numRows; ++row) {
		for (int col = 0; col < numCols; ++col) {
			int btnIdx = (row * numCols) + col;
			if (btnIdx >= numBtns) { continue; }
			ss.str("");		ss << buttonNames[btnIdx];
			vector<float> v1 = { col*perBtnWidth, (row * (btnHigh + initY)), btnWide, btnHigh, .75f, .75f, .75f, 1 };
			UIList[0]->initUIObj(idx++, ss.str(), 0, v1, objType, objLabels, objXY_WH, objClr);
		}
	}
	//needs to be done after buttons made - assumes last button made is showUI button.
	showUIBtnIdx = idx - 1;		
	UIList[0]->setUI(objType, objLabels, objXY_WH, objClr);

	UIList[0]->height = (numRows * (btnHigh + initY));

	for (int i = 0; i < UIList[0]->MyPgUIBtn.size(); ++i) { UIList[0]->MyPgUIBtn[i]->flags[UIList[0]->MyPgUIBtn[i]->UIobjIDX_CtrLbl] = true; }//center button captions
}
//go through the UI objects, find what has been clicked
bool MyWindow::handleBtnClick() {
	bool result = false;
	for (int i = 0; i < UIList[0]->MyPgUIBtn.size(); i++) {
		if (UIList[0]->MyPgUIBtn[i]->isClicked()) {
			std::string onclick = UIList[0]->MyPgUIBtn[i]->getOnClick();
			cout << "MyWindow Button Clicked ID:" << i << "|" << UIList[0]->MyPgUIBtn[i]->getLabel() << " onclick = " << onclick << endl;
			switch (i) { // 2-4 and beyond should be in order of flag idx's
				case 0: {toggleSimState(); glutPostRedisplay(); return true; }				//"Start/Stop Sim",
				case 1: {reinitSim(); return true; }					//"Reset Sim"
				case 2: {setFlag(debug, !_wSM[debug], i);  return true; }				//toggle debug mode
				case 3: {setFlag(showSkels, !_wSM[showSkels], i);  return true; }		//show/hide skeleton walkers for all sim threads
				case 4: {setFlag(showUI, !_wSM[showUI], i);  return true; }				//show/hide UI for all components
				case 5: {setFlag(showSimCnst, !_wSM[showSimCnst], i);  return true; }				//show/hide UI for all components
				case 6: {setFlag(testSimDet, !_wSM[testSimDet], i);  return true; }				//show/hide UI for all components
				case 7: {setFlag(selMrkr, !_wSM[selMrkr], i);  return true; }				//enable and show/disable and hide skeleton handles/markers  (_wSM[selMrkr]) 
				default: {return false; }
			}
		}//if click
	}//for each button
	return false;
}

void MyWindow::reinitSim() {
	//cycle through all UI elements to reinitialize any who have values that could have been modified
	mSimulating = false;
	_wSM[beginCPBP] = false;								//set false since moving to initial starting position
	UIList[0]->MyPgUIBtn[0]->label = "Start Sim";			//
	initFlags();
	cpbp->resetAllCntxts();
	cpbp->initCPBPHandler();				//this remakes all skeletons
	initSkels();
	cpbp->masterSimCntxt->setStateAsRestState(initState, true, false);
	cpbp->setMasterPoseAsRestState(false);
}

void MyWindow::initSkels() {
	//main_skel->setPositionSegment(genCoordIds, initConfig);
	//main_skel->computeForwardKinematics(true, true, false);
	//for (int i = 0; i < cpbp->mFwdSimCntxts.size(); ++i) {
	//	cpbp->mFwdSimCntxts[i]->s->setPositionSegment(genCoordIds, initConfig);
	//	cpbp->mFwdSimCntxts[i]->s->computeForwardKinematics(true, true, false);
	//}
	main_skel->setPositions(genCoordIds, initConfig);
	main_skel->computeForwardKinematics(true, true, false);
	for (int i = 0; i < cpbp->mFwdSimCntxts.size(); ++i) {
		cpbp->mFwdSimCntxts[i]->s->setPositions(genCoordIds, initConfig);
		cpbp->mFwdSimCntxts[i]->s->computeForwardKinematics(true, true, false);
	}

}

//debug stuff from timestepping()
//cpbp->masterSimCntxt->ownWorld->step();			//master context world
//cpbp->masterSimCntxt->updateStateControl();
//if (_wSM[beginCPBP]) {//we've started cpbp and iterated at least 1 time
//	double varLevel = 0;
//	mainCntxtCost = 0;
//	for (int i = 0; i < cpbp->numGoals; ++i) {			mainCntxtCost += cpbp->goals[i]->calcFrmSqCost(cpbp->masterSimCntxt, varLevel, cpbp->masterSimCntxt->elapsedSimTime);		}
//	UIList[0]->MyPgUICaption[0]->label = UIList[0]->MyPgUICaption[0]->buildLabel(true, mainCntxtCost, "%.4f");
//}

//genCoordIds.push_back(1);	// root rot around y axis
//genCoordIds.push_back(4);	// root location in y
//genCoordIds.push_back(6);   // left hip
//genCoordIds.push_back(9);   // left knee
//genCoordIds.push_back(10);  // left ankle
//genCoordIds.push_back(13);  // right hip
//genCoordIds.push_back(16);  // right knee
//genCoordIds.push_back(17);  // right ankle
//genCoordIds.push_back(21);  // lower back
//initConfig.setZero();
//initConfig.head(genCoordIds.size()) << -0.2, -0.05, 0.15, -0.4, 0.25, 0.15, -0.4, 0.25, 0.0;



////debug stuff : test of all dofs having same orientation for position and for velocity motors
//Vector3d gravity(0.0,0.0, 0.0);		//this is to test angle limits
//cpbp->masterSimCntxt->ownWorld->setGravity(gravity);
//dispCtr += .001;
//double val = M_PI *.5* std::sin(dispCtr);
//double mtrVal = (val > 0 ? 3 : -3);
//int numIters = (cpbp->masterSimCntxt->numDofs < cpbp->mFwdSimCntxts.size() ? cpbp->masterSimCntxt->numDofs : cpbp->mFwdSimCntxts.size());
//for (int i = 0; i < numIters; ++i) {//need 30 skels to do all of these
//	cpbp->mFwdSimCntxts[i]->ownWorld->setGravity(gravity);
//	if (dispVelOrDir) {	cpbp->mFwdSimCntxts[i]->s->setCommand(i + 6, mtrVal);cout << "Set joint mtr vel for cntxt : " << cpbp->mFwdSimCntxts[i]->name<<" : " << cpbp->mFwdSimCntxts[i]->dofNames[i + 6] << " to be :" << mtrVal << endl;}
//	else {cpbp->mFwdSimCntxts[i]->s->setPosition(i + 6, val);	cout << "Set joint pos for cntxt : " << cpbp->mFwdSimCntxts[i]->name << " : " << cpbp->mFwdSimCntxts[i]->dofNames[i + 6] << " to be :" << val << endl;}
//	cpbp->mFwdSimCntxts[i]->ownWorld->step();
//}
//if (dispVelOrDir) {cpbp->masterSimCntxt->s->setCommand(36, mtrVal);cout << "Set joint mtr vel for cntxt : " << cpbp->masterSimCntxt->name << " : " << cpbp->masterSimCntxt->dofNames[36] << " to be :" << mtrVal << endl;}
//else {	cpbp->masterSimCntxt->s->setPosition(36, val);	cout << "Set joint pos for cntxt : " << cpbp->masterSimCntxt->name << " : " << cpbp->masterSimCntxt->dofNames[36] << " to be :" << val << endl;}
//cpbp->masterSimCntxt->ownWorld->step();