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
#ifndef APPS_PARTICLEBELIEFPROP_MYWINDOW_H_
#define APPS_PARTICLEBELIEFPROP_MYWINDOW_H_

#include "dart/gui/SimWindow.h"
#include "CPBPHandler.h"
#include "CPBPParams.h"
#include "SimContext.h"
#include <string>
#include <fstream>
#include "dart/dart.h"
#include "ControlPBP.h"
#include "MyGuiHandler.h"
#include "CPBPParser.h"


using namespace AaltoGames;
using namespace cPBPropApp;

class MyWindow : public dart::gui::SimWindow {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	MyWindow(void);
	virtual ~MyWindow() {}
	//override all default behavior
	void initWindow(int _w, int _h, const char* _name);
	//build structure holding all skeletons/worlds for multithreaded operation
	void initSkelsAndWin(int _w, int _h);
	//initialize all UI constructs
	void initUIPages();
	//set initial state of flags
	void initFlags();		

	void reinitSim();
	//event handler for UI components related to this class
	bool handleBtnClick();
	//build the UI construct to modify the values of this class
	void buildUI();
	//initialize UI for this window
	void initUI() { buildUI(); }
	//first running cpbp algorithm (upon first touching the ground)
	//void initialCPBPRun();
	//ui functions
	void drawVecBall(GLUquadricObj *c, GLdouble rad, GLint sl, GLint st, GLfloat cr, GLfloat cg, GLfloat cb, GLfloat ca, const Eigen::Vector3d& coord);
	void drawVecLine(GLfloat cr, GLfloat cg, GLfloat cb, GLfloat ca, const Eigen::Vector3d& start, const Eigen::Vector3d& end);
	void drawFrcVecLine(GLfloat cr, GLfloat cg, GLfloat cb, GLfloat ca, const Eigen::Vector3d& start, const Eigen::Vector3d& end);
	void drawAxes(const Eigen::Vector3d& axesLoc, float len, bool altColor);
	void drawCOM(const Eigen::Vector3d& com, const Eigen::Vector3d& cop, const Eigen::Vector3d& comVel, double grnd);
	//void drawStringOnScreen(float x, float y, const std::string& s);
	void drawSelMrkrDragForce(GLUquadricObj *c);
	void drawTextAtLocation(float x, float y, string text, float lineW, float scl);

	void displayDebugTestInfo(int idx, int cntrlIdx, int testIter);

	void drawCostAndResamp();

	inline string buildStrFromFloat(float val, const char* fmt = "%.4f") {
		char buf[MAX_BUF];
		stringstream ss;
		sprintf(buf, fmt, val);
		ss << buf;
		return ss.str();// label = ss.str();
	}//buildLabel
	inline string buildStrFromDbl(double val, const char* fmt = "%.4f") {
		char buf[MAX_BUF];
		stringstream ss;
		sprintf(buf, fmt, val);
		ss << buf;
		return ss.str();// label = ss.str();
	}//buildLabel
	void getCurColor();
	void setOldColor();
	void drawFwdWalkers();
	void drawJointAxes();
	void drawJointAxis(dart::dynamics::BodyNode* node); 

	void setSimState(bool _ns);
	void toggleSimState() { setSimState(!mSimulating); }
	void setFlag(int idx, bool _val, int _btnIdx = -1);
	void drawWalkerTraj();

	void setShowUI(bool show) {
		for (int i = 0; i < UIList.size(); ++i) {			UIList[i]->setShowUI(show);		}
	}

	void testSimCntxtDet();

	std::string buildStrFromEigen3d(const Eigen::Ref<const Eigen::Vector3d>& vec) { stringstream ss;  ss << buildStrFromDbl(vec(0)) << "," << buildStrFromDbl(vec(1)) << "," << buildStrFromDbl(vec(2)); return ss.str(); }
	std::string cntctToString(dart::collision::Contact& cntct) {
		stringstream ss;
		try {
			auto p = cntct.bodyNode1.lock();
			ss << "Contact Between b1:" << p->getName();
		}
		catch (bad_weak_ptr b) {
			ss << "Contact Between b1 : bad weak pointer for bodynode1 ";
		}
		try {
			auto p = cntct.bodyNode2.lock();
			ss << " and b2:" << p->getName();
		}
		catch (bad_weak_ptr b) {
			ss << "and b2 : bad weak pointer for bodynode2 ";
		}
		ss << " | pt : " << buildStrFromEigen3d(cntct.point)
			<< " | f : " << buildStrFromEigen3d(cntct.force)
			<< " | n : " << buildStrFromEigen3d(cntct.normal)
			<< " | pene : " << buildStrFromDbl(cntct.penetrationDepth)
			<< " | force in y : " << buildStrFromDbl(cntct.normal.dot(cntct.force));
		return ss.str();
	}

	void initSkels();
	virtual void displayTimer(int _val) override;
	virtual void timeStepping();
	//draw UI on screen
	void draw2dUI();
	//toggle 2d ui mode or 3d object mode in drawing
	void enable2dUIMode(bool on);
	virtual void draw();
	virtual void drawSkels();
	virtual void keyboard(unsigned char _key, int _x, int _y);
	int coordsToMarker(int _x, int _y);
	int processHits(GLint _hits, GLuint _buffer[]);
	//set up "constraint" locations to create pull force (from marker to mouse location)
	void applyMarkerPullForce();
	void createConstraint(int idx);
	void modifyConstraint(const Eigen::Vector3d& _deltaP);
	void removeConstraint(int idx);
	void clearAllConstraints();
	Eigen::Vector3d reverseProjection(double _x, double _y);

	virtual void click(int _button, int _state, int _x, int _y);
	virtual void drag(int _x, int _y);
	virtual void move(int _x, int _y);
	//check through all UI pages for where a click was registered
	int checkPagesForClick(int _button, int _eventType, int& _x, int& _y);

public : //variables
	Eigen::Quaterniond origTrackBallQ;									//original eye location             
	float cc[4];														//original colors - reset to override local changes in draw
	std::shared_ptr<CPBPHandler> cpbp;													//handle the control pbp algorithm

	std::vector<size_t> genCoordIds;
	Eigen::VectorXd initConfig;
	Eigen::VectorXd initState;											//initial state of skeleton - for resetting configuration
	
	int activeMarkerCnt;												//# of active markers
	Eigen::Vector3d clickTarget;										//location of current click re: marker dragging
	std::vector<bool> selMrkrIDXs;										//whether or not marker[idx] is selected for movement
	std::vector<std::string> frcAppStr;											//the most recent force applied by dragging markers

	dart::dynamics::SkeletonPtr main_skel;
	std::shared_ptr<CPBPParams> cp;																//modfiable params and hyperparams

	std::vector<std::shared_ptr<MyGuiHandler>> UIList;					//all UI components being displayed
	int showUIBtnIdx;													//idx in this window's UIComponents of the button to show/hide UI - needed so button can always be shown and processed
	//for timerfunction callback, to override default dart behavior and allow for timestep bigger than callback
	double elapsedSimTime;												//time elapsed since initial timestepping call	

	double mainCntxtCost;												//per-frame cost calc of main context
	vector<bool> _wSM;													
	static const int beginCPBP = 0;										//whether the cpbp algorithm has begun iterating in the time step function - to allow the skeleton to hit the ground before cpbp starts.  always true once skel hits ground
	//driven by UI buttons
	static const int debug = 1;											//debug mode - show/hide debug info
	static const int showSkels = 2;										//whether we should show final state of walker skels
	static const int showUI = 3;										//show/hide UI components
	static const int showSimCnst = 4;									//whether to show sim contxt costs or prev idx's
	static const int testSimDet = 5;									//run test on sim cntxts, to see if they all respond the same way when they get the same initial state an the same series of controls.
	static const int selMrkr = 6;										//whether marker selection/pulling is enabled
	static const int dragMrkr = 7;										//whether we are currently dragging a marker (creating a force from selected marker(s) to mouse location
	static const int altPressed = 8;									//whether or not the alt key is currently pressed

	const int numFlags = 9;
};

#endif  // APPS_PARTICLEBELIEFPROP_MYWINDOW_H_
