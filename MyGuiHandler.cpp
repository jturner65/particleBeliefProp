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

#include "MyGuiHandler.h"
#include "dart/dart.h"
#include <memory>
#include <Eigen/Dense>
#include <iostream>

namespace cPBPropApp {

	const float MyGuiHandler::sldrBarLen = 140;

	MyGuiHandler::MyGuiHandler(float _stX, float _stY, double _w, double _h) :stX(_stX), stY(_stY), winX(_w), winY(_h), oldMousePos(-1, -1, -1), numObjs(0), width(10), height(10), showUI(true)
	{ initUI(); }
	MyGuiHandler::~MyGuiHandler() {}

	void MyGuiHandler::initUI() {
		//any initialization necessary
	}

	void MyGuiHandler::initUIObj(int idx, std::string label, int type, std::vector<float>& vals, std::vector<int>& objType, std::vector<std::string>& objLabels, std::vector<std::vector<int>>& objXY_WH, std::vector<std::vector<float>>& objClr) {
		std::vector<int> tmpVec(4);
		std::vector<float> tmpVec2(4);
		for (int i = 0; i < 4; ++i) { tmpVec[i] = vals[i]; tmpVec2[i] = vals[i + 4]; }
		//tmpVec[0] = x; tmpVec[1] = y; tmpVec[2] = w; tmpVec[3] = h;
		//tmpVec2[0] = r; tmpVec2[1] = g; tmpVec2[2] = b; tmpVec2[3] = a;
		objType[idx] = type;
		objXY_WH[idx] = tmpVec;
		objLabels[idx] = label;
		objClr[idx] = tmpVec2;
	}

	//build UI with passed values represnenting a list of UI objects, sizes, locations, and colors
	void MyGuiHandler::setUI(std::vector<int>& objType, std::vector<std::string>& objLabels, std::vector<std::vector<int>>& objXY_WH, std::vector<std::vector<float>>& objClr) {
		setUI(objType, objLabels, objXY_WH, objClr, MyGuiHandler::sldrBarLen);
	}
	void MyGuiHandler::setUI(std::vector<int>& objType, std::vector<std::string>& objLabels, std::vector<std::vector<int>>& objXY_WH, std::vector<std::vector<float>>& objClr, float _barLen) {
		for (int i = 0; i < numObjs; ++i) {
			switch (objType[i]) {
				case 0: {MyUIButton* cmp = setUIButton(objLabels[i], objXY_WH[i], objClr[i]);	addUIButton(cmp);  break; }//button
				case 1: {MyUISlider* cmp = setUISlider(objLabels[i], objXY_WH[i], objClr[i], _barLen);	addUISlider(cmp); break; }//slider
				case 2: {MyUICaption* cmp = setUICaption(objLabels[i], objXY_WH[i], objClr[i]); addUICaption(cmp); break; }//caption
				case 3: {MyUITextBox* cmp = setUITextBox(objLabels[i], objXY_WH[i], objClr[i]); addUITextBox(cmp);  break; }//textbox
				default: {MyUICaption* cmp = setUICaption(objLabels[i], objXY_WH[i], objClr[i]); addUICaption(cmp); break; }//def to caption
			}//switch
		}
	}
	//for each type being supported
	MyUIButton* MyGuiHandler::setUIButton(std::string& label, std::vector<int>& objXY_WH, std::vector<float>& objClr) {
		MyUIButton* cmp = new MyUIButton(0, 0);
		setBaseVals(cmp, label, objXY_WH, objClr);
		return cmp;
	}

	MyUISlider* MyGuiHandler::setUISlider(std::string& label, std::vector<int>& objXY_WH, std::vector<float>& objClr, float _barLen) {
		MyUISlider* cmp = new MyUISlider(0, 0);
		setBaseVals(cmp, label, objXY_WH, objClr);
		int sdiv = 0,
			sbarlen = _barLen,
			sbarthick = 8;
		float srange = 100, spos = 0;
		std::string dir = "h", ltor = "t";
		cmp->setIsHoriz((dir == "1" || dir == "h" || dir == "horizontal"));
		cmp->setIsLToR((ltor == "1" || ltor == "t" || ltor == "true"));                                //slider increases left to right(horiz), or top to bottom(vert), or slider increases right to left(horiz), or bottom to top(vert),
		cmp->setSlideVals(srange, spos, sdiv, sbarlen, sbarthick);
		double min = 0.0, max = 1.0;
		cmp->setSlideMin(min);
		cmp->setSlideRng(max - min);
		return cmp;
	}
	MyUITextBox* MyGuiHandler::setUITextBox(std::string& label, std::vector<int>& objXY_WH, std::vector<float>& objClr) {
		MyUITextBox* cmp = new MyUITextBox(0, 0);
		setBaseVals(cmp, label, objXY_WH, objClr);
		cmp->setMaxChars(10);
		return cmp;
	}
	MyUICaption* MyGuiHandler::setUICaption(std::string& label, std::vector<int>& objXY_WH, std::vector<float>& objClr) {
		MyUICaption* cmp = new MyUICaption(0, 0);
		setBaseVals(cmp, label, objXY_WH, objClr);
		return cmp;
	}
	//shared values for all objs
	void MyGuiHandler::setBaseVals(MyUIComponent* cmp, std::string& label, std::vector<int>& objXY_WH, std::vector<float>& objClr) {
		cmp->winX = winX;
		cmp->winY = winY;
		cmp->setLoc(objXY_WH[0], objXY_WH[1]);
		cmp->setDim(objXY_WH[2], objXY_WH[3]);
		cmp->setLabel(label);
		cmp->setMSOverTxt(label);
		cmp->setColor(objClr[0], objClr[1], objClr[2], objClr[3]);
	}
	void MyGuiHandler::addUIButton(MyUIButton* btn) {
		std::shared_ptr<MyUIButton>    tmp0(btn);
		std::shared_ptr<MyUIComponent> tmp1 = tmp0;
		MyPgUIBtn.push_back(tmp0);
		MyPgUICmp.push_back(tmp1);
	}

	void MyGuiHandler::addUISlider(MyUISlider* sld) {
		std::shared_ptr<MyUISlider>    tmp0(sld);
		std::shared_ptr<MyUIComponent> tmp1 = tmp0;
		MyPgUISldr.push_back(tmp0);
		MyPgUICmp.push_back(tmp1);
	}
	void MyGuiHandler::addUITextBox(MyUITextBox* tbx) {
		std::shared_ptr<MyUITextBox>    tmp0(tbx);
		std::shared_ptr<MyUIComponent> tmp1 = tmp0;
		MyPgUITextBox.push_back(tmp0);
		MyPgUICmp.push_back(tmp1);
	}

	void MyGuiHandler::addUICaption(MyUICaption* cap) {
		std::shared_ptr<MyUICaption>    tmp0(cap);
		std::shared_ptr<MyUIComponent> tmp1 = tmp0;
		MyPgUICaption.push_back(tmp0);
		MyPgUICmp.push_back(tmp1);
	}
	//draw red box around UI region
	void MyGuiHandler::drawBoundBox() {
		glPushMatrix();
		glTranslatef(stX, stY, 0);
		glColor4f(1, 0, 0, 1);
		glBegin(GL_LINE_LOOP);
		glVertex3f(0, 0, 0);
		glVertex3f(width, 0, 0);
		glVertex3f(width, height, 0);
		glVertex3f(0, height, 0);
		glEnd();

		glPopMatrix();
	}

	void MyGuiHandler::drawObj(int idx) {
		glPushMatrix();
		glTranslatef(stX, stY, 0);
		if (MyPgUICmp[idx]->flags[MyUIComponent::UIobjIDX_Display]) { MyPgUICmp[idx]->MyUIComponent::draw(); }
		glPopMatrix();
	}

	//called from myWindow
	void MyGuiHandler::drawObjs() {
		glPushMatrix();
		glTranslatef(stX, stY, 0);
		for (int i = 0; i < MyPgUICmp.size(); ++i) {
			glPushMatrix();
			if (MyPgUICmp[i]->flags[MyUIComponent::UIobjIDX_Display]) { MyPgUICmp[i]->MyUIComponent::draw(); }
			glPopMatrix();
		}
		glPopMatrix();
	}

	//set button to be show-hide button by idx in MyPgUIBtn
	void MyGuiHandler::setShowHideBtn(int idx) {
		showHideBtn = MyPgUIBtn[idx];
		showHideBtn->flags[MyUIComponent::UIobjIDX_NeverHide] = true;
	}
	//turn on or off UI
	void MyGuiHandler::setShowUI(bool show) {
		showUI = show;
		for (int i = 0; i < MyPgUICmp.size(); ++i) {
			MyPgUICmp[i]->setShown(show);
		}
		showHideBtn->label = (show ? "Hide UI" : "Show UI");
	}


	//called from myWindow - btn currently ignored
	//event : 2 is drag, 1 is click down, 0 is click release
	bool MyGuiHandler::checkUIElements(int& xp, int& yp, int& d, bool mouseEvent, int btn, int evnt) {
		//need to be in local frame of reference, so subtract _stX,  _stY from x and y
		//if (!showPage) { return false; }					//if not shown, do not perform any UI capabilities
		int x = xp - stX, y = yp - stY;
		bool objFound = false;
		//mouse events : mouse action has triggered event
		bool msDragEvent = (2 == evnt) ? true : false;															//drag
		bool msClickEvent = (1 == evnt) ? true : false;
		bool msReleaseEvent = (0 == evnt) ? true : false;
		int m = MyPgUICmp.size();
		std::shared_ptr<MyUIComponent> cmp;
		for (int idx = 0; idx < m; ++idx) {			
			cmp = MyPgUICmp[idx];
			int clkVal = (cmp->isInside(x, y));                                                      //value returned from isInside - what part of object click was inside
			if ((0 != clkVal) || (cmp->checkDragByThisCntrl(UI_MOUSE))) {                             //check if inside this object, or if this object is currently being dragged (check inside also checks if displayed)
				if (!msDragEvent) {
					if (msClickEvent) {
						cmp->setClicked(x, y, clkVal, UI_MOUSE);
					}
					else {
						cmp->clearClicked(x, y, UI_MOUSE);
					}                                   //upon click up clear click event - needs to happen cycle after "click release" has been processed
				}
				else {
					int resD = cmp->setDragged(msDragEvent, x, y, UI_MOUSE);
					if (resD != -1) { cmp->stickToClick(x, y); }
				}										                                                                //set drag state to be whether or not this is a drag event						
				objFound = true;
			}
			else { 
				cmp->clearClicked(x, y, UI_MOUSE); 
			}									                    //use location = -1,-1 for click release of object with mouse outside of object (location ignored)
		}
		oldMousePos = Eigen::Vector3f(xp, yp, 0);															//save mouse position in world UI coords
		return objFound;
	}//checkUIElements  

	unsigned int MyUIComponent::ID_gen = 0;

	//specifically for being re-inited by external code for 
	int MyUIComponent::reInit() {
		//any external-specific functionality
		init();
		return 0;
	}

	void MyUIComponent::init() {
		flags.resize(UIobj_numFlags, false);
		UIvalue.resize(UIobj_numVals);
		//initial values for various properties
		flags[UIobjIDX_Display] = true;
		flags[UIobjIDX_CanClick] = true;
		flags[UIobjIDX_CanDispMO] = true;
		
		lineWt = 1.1f;  //override this for any component that needs to change it, by type, in component's init
	}

	//draws button component of slider, crank and button 
	void MyUIComponent::drawBaseCmp(float delX, float delY, float lineW) {
		glPushMatrix();
		if (flags[UIobjIDX_MSDown]) { glColor4f(.5f - clr[0] / 2.0f, .5f - clr[1] / 2.0f, .5f - clr[2] / 2.0f, clr[3]); }
		else { glColor4f(clr[0] / 2.0f, clr[1] / 2.0f, clr[2] / 2.0f, clr[3]); }

		glTranslatef(x, y, 0);
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINES);
		glLineWidth(1.0f);
		glBegin(GL_QUADS);
		glVertex2f(0, 0);
		glVertex2f(w, 0);
		glVertex2f(w, h);
		glVertex2f(0, h);
		glEnd();

		if (flags[UIobjIDX_MSDown]) { glColor4f(1.0f - clr[0], 1.0f - clr[1], 1.0f - clr[2], clr[3]); }
		else { glColor4f(clr[0], clr[1], clr[2], clr[3]); }
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glBegin(GL_QUADS);
		glVertex2f(2, 2);
		glVertex2f(w - 4, 2);
		glVertex2f(w - 4, h - 4);
		glVertex2f(2, h - 4);
		glEnd();
		glPopMatrix();
		drawText(delX, delY, label, lineW);
	}//drawBaseCmp

	void MyUIComponent::drawText(float delX, float delY, std::string text, float lineW, bool manScale, float scH) {
		if (flags[UIobjIDX_OffsLbl]) { drawTextAtLocation(x + delX, y + delY, text, lineW, manScale, scH); }
		else { drawTextAtLocation(x, y, text, lineW, manScale, scH); }
	}//drawText

	void MyUIComponent::drawTextAtLocation(float x, float y, std::string text, float lineW, bool manScale, float scH) {
		glPushMatrix();		
		glTranslatef(x, y, 0);
		glColor4f(0, 0, 0, 1);
		glLineWidth(lineW);			//default to 1.25f ?
		int lblLen = text.length();
		int chrStrkLen = 0;                                                         //length of glut stroke chars in "units" - use to dtermine size scaling
		float scVal;
		for (int i = 0; i < lblLen; ++i) { chrStrkLen += glutStrokeWidth(GLUT_STROKE_ROMAN, text[i]); }
		if (manScale) { scVal = scH; }                                                  //if passing manual scaling amount, use this, otherwise calculate scale amount			
		else {                                                                           //if not manual scaling - calculate appropriate scaling to keep text within ui component boundaries (w x h) 
			float val = (w > h ? h : w);
			float maxV = (.95f * val / chrStrkLen > UIobj_MinFontScale ? .95f * val / chrStrkLen : UIobj_MinFontScale);
			scVal = (maxV < UIobj_MaxFontScale ? maxV : UIobj_MaxFontScale);
		}
		if (flags[UIobjIDX_CtrLbl]) {	glTranslatef((w - (chrStrkLen * scVal)) / 2.0f, (h + (h * scVal)) / 2.0f, 0);	}	 //cap letters are approx 100 pxls high before scaling      
		glScalef(scVal, -scVal, 1);
		for (int i = 0; i < lblLen; ++i) { glutStrokeCharacter(GLUT_STROKE_ROMAN, text[i]); }
		glPopMatrix();

	}//drawText

	//handles ui event
	int MyUIComponent::click(float _x, float _y, int cmpObj) {
		int res = -1;
		if ((_x != -1) && (_y != -1)) {
			stickX = _x - x;                                                    //click location relative to center of object - stick control to this relative location as ui component moves
			stickY = _y - y;
			res = this->click(_x, _y, cmpObj);								    //object specific handler - add 1 for obj 0
			this->clkX = _x; this->clkY = _y;									//set click location, for next round
		}
		//else {   }
		return res;
	}//click

	int MyUIComponent::drag(float _x, float _y) {
		int res = this->drag(_x, _y);							                //object specific handler
		this->clkX = _x; this->clkY = _y;							            //set click location, for next round
		return res;
	}

	std::ostream& operator<<(std::ostream& out, const MyUIComponent& cmp) {
		out << "ID : " << cmp.ID << " label : " << cmp.label << " onClick : " << cmp.onclick << "\n";
		out << "location : (" << cmp.x << ", " << cmp.y << ") dim : (" << cmp.w << ", " << cmp.h << ")" << "\n";
		out << "Color : (r: " << cmp.clr[0] << " g: " << cmp.clr[1] << " b: " << cmp.clr[2] << " a: " << cmp.clr[3] << ") " << "\n";
		out << "Flags : Display : " << cmp.flags[cmp.UIobjIDX_Display] << " Can Click : " << cmp.flags[cmp.UIobjIDX_CanClick] << " Can Drag : " << cmp.flags[cmp.UIobjIDX_CanDrag] << "\n";
		return out;
	}

	int MyUISlider::reInit() {
		MyUIComponent::reInit();		//parent reinit/init
		init();
		return 0;
	}
	void MyUISlider::init() {
		//initial values for various properties
		flags[UIobjIDX_CanDrag] = true;
		dispMinMax = true;
	}
	//draw this component on the screen
	void MyUISlider::draw() {
		float trnX, trnY, lblX, lblY,								        //location of slider thumb, based on value of slider,with 0 putting at top/left, and 1 placing at bottom/right, and label above thumb
			lenX, lenY, trX, trY;

		glPushMatrix();
		drawSliderBar();
		glPopMatrix();
		glPushMatrix();
		if (horiz) {
			lenX = sbDimL;				//dimensions of slider based on orientation
			lenY = sbDimTh;
			trX = x;		//location of slider bar based on orientation
			trY = y + (abs(h - sbDimTh) / 2.0f);
			trnX = -(w / 2.0) + slidePos * sbDimL;
			trnY = 0;
			lblX = 0;                            //move label above thumb if horizontal, to the right if vertical         
			lblY = -(3 * h / 5.0);
		}
		else {
			trnX = 0;
			trnY = -(h / 2.0) + (slidePos * sbDimL);
			lblX = (3 * w / 5.0);                               //move label above thumb if horizontal, to the right if vertical         
			lblY =  0;
			lenX = sbDimTh;					//dimensions of slider based on orientation
			lenY = sbDimL;
			trX = x + (abs(w - sbDimTh) / 2.0f);			//location of slider bar based on orientation
			trY = y;
		}
		glTranslatef(trnX, trnY, 0);                                    //move to location for button based on slider progress
		if (useCaption) {
			this->labelHolder->label = this->labelHolder->buildLabel( this);
			MyUIComponent::drawBaseCmp(lblX, lblY, lineWt);
		}//, MyUIComponent* cmp
		else {
			this->label = this->buildLabel(true, this);
			MyUIComponent::drawBaseCmp(lblX, lblY, lineWt);
		}
		glPopMatrix();
		if (dispMinMax) {			//display min and max vals at ends of slider
			glPushMatrix();
			std::string minV = buildLabel(false, slideMin, "%.2f"), maxV = buildLabel(false, slideMin + slideRng, "%.2f");
			int mn = minV.size();
			glTranslatef(trX - (3.5 * mn), trY, 0);
			MyUIComponent::drawTextAtLocation(0, .8f*lenY, minV, lineWt, true, .06f);
			MyUIComponent::drawTextAtLocation(lenX + (3.6*mn), .8f*lenY, maxV, lineWt, true, .06f);
			glPopMatrix();
		}
	}//draw
	//draw the rails the slider button operates on
	void MyUISlider::drawSliderBar() {
		float lenX, lenY, trX, trY, frm = 1;
		if (horiz) {
			lenX = sbDimL ;				//dimensions of slider based on orientation
			lenY = sbDimTh;
			trX = x ;		//location of slider bar based on orientation
			trY = y + (abs(h - sbDimTh) / 2.0f);
		}
		else {
			lenX = sbDimTh;					//dimensions of slider based on orientation
			lenY = sbDimL;
			trX = x + (abs(w - sbDimTh) / 2.0f);			//location of slider bar based on orientation
			trY = y;
		}
		//glTranslatef(x,y,0);
		glPushMatrix();
		glTranslatef(trX, trY, 0);
		glColor4f(0, 0, 0, 1);
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINES);
		glLineWidth(1.0f);
		glBegin(GL_QUADS);
		glVertex2f(0, 0);
		glVertex2f(lenX, 0);
		glVertex2f(lenX, lenY);
		glVertex2f(0, lenY);
		glEnd();

		glColor4f(.4f, .4f, .4f, 1);
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINES);
		glLineWidth(1.0f);
		glBegin(GL_QUADS);
		glVertex2f(frm, frm);
		glVertex2f(lenX - frm, frm);
		glVertex2f(lenX - frm, lenY - frm);
		glVertex2f(frm, lenY - frm);
		glEnd();
		glPopMatrix();		
	}
	//handles ui event - val gets value put into it
	void MyUISlider::getValueForEvent(int hand, int type, int drag, float& val) {}
	//if click in scroll bar (cmpObj == 2) then move "thumb"/indicator toward click location
	int MyUISlider::click(float _x, float _y, int cmpObj) {
		if (horiz) {
			if (2 == cmpObj) { slidePos = (_x - x) / sbDimL;	capSlidePos(); }
			stickX -= int(-(w / 2.0) + slidePos * sbDimL);
		}
		else {
			if (2 == cmpObj) { slidePos = (_y - y) / sbDimL;	capSlidePos(); }           //move thumb due to click happening in scroll bar
			stickY -= int(-(h / 2.0) + (slidePos * sbDimL));
		}
		return ID;
	}
	//drag thumb around
	int MyUISlider::drag(float _x, float _y) {
		float del = ((horiz) ? _x - clkX : _y - clkY) * UIobj_DragSens;					                            //sensitivity is modifiable
		slidePos += del / (1.0f*sbDimL);
		// cout<<"slider mod amt : "<< del/(1.0f * sbDimL)<<" del : "<<del<<" sbDimL "<<(sbDimL)<<" x,y : ("<<x<<", "<<y<<") _x,_y : ("<<_x<<", "<<_y<<") clkX,clkY : ("<<clkX<<", "<<clkY<<")"<<"\n";
		capSlidePos();
		return ID;
	}
	void MyUISlider::setCaption(std::shared_ptr<MyUICaption> _cap, std::shared_ptr<MyUISlider> _sldr) {
		labelHolder = _cap;
		_cap->sliderOwner = _sldr;
		useCaption = true;
	}
	std::ostream& operator<<(std::ostream& out, const MyUISlider& cmp) {
		out << static_cast<const MyUIComponent&>(cmp);                                                                        //start with base class
		out << "Slider specific attribs : Min Val : " << cmp.slideMin << " Range : " << cmp.slideRng << " Current position (% of range) : " << (cmp.slidePos * 100) << "% " << "\n";
		out << "\t# of subdivisions of range : " << cmp.slideSubdiv << " Slider track dimensions (Len,Thk) : (" << cmp.sbDimL << ", " << cmp.sbDimTh << ") Orientation : " << (cmp.horiz ? "Horizontal " : "Vertical ") << "\n";
		out << "\tIncreasing value dir : " << (cmp.horiz ? (cmp.LToR ? "Left to Right" : "Right To Left") : (cmp.LToR ? "Down to Up" : "Up to Down")) << "\n";
		out << "\tUse list for values : " << cmp.useListVals << "\n";
		if (cmp.useListVals) {
			out << "\tDisplay List Values :";
			for (int i = 0; i < cmp.dispListVals.size(); ++i) { out << (i == 0 ? " `" : ", `") << cmp.dispListVals[i] << "`"; }
			out << "\n";
			out << "\tReturned List Values : ";
			for (int i = 0; i < cmp.resListVals.size(); ++i) { out << (i == 0 ? " `" : ", `") << cmp.resListVals[i] << "`"; }
			out << "\n";
		}
		return out;
	}//op<<

	int MyUITextBox::reInit() {			//any external-specific functionality
		MyUIComponent::reInit();		//parent reinit/init
		init();
		return 0;
	}

	void MyUITextBox::init() {
		flags[UIobjIDX_CanType] = true;
		flags[UIobjIDX_OffsLbl] = true;//always offset label for textboxes
		txtVal = "";
		newTxtVal = "";
		tbState = 0;
		beingEdited = false;
		cursorOn = false;
		maxChars = 10;  //default accept 10 chars
	}
	//current textbox text displayed, put cursor at end of text, accept new text. 
	//esc from ch ignores input, enter processes it by adding it to existing txtVal string
	//newTxtVal will enter this for the first time having a copy of the existing text in the textbox
	bool MyUITextBox::acceptInput(unsigned char ch) {
		switch (ch) {
			case(27) : {//esc key - discard all new text entered - retain old txtVal
				newTxtVal = "";
				beingEdited = false;                //end editing
				cursorOn = false;
				break; }
			case(13) : {//enter pressed - save modified text as the txtVal
				txtVal = std::string(newTxtVal);
				newTxtVal = "";
				beingEdited = false;                //end editing
				cursorOn = false;
				break; }
			case(8) :            //backspace
			case(127) : {         //delete
				if (0 < newTxtVal.length()) {//remove char from end of newTxtVal if there are chars to remove
					newTxtVal = std::string(newTxtVal, 0, newTxtVal.length() - 1);
				}
				break; }
			default: {
				if (maxChars > newTxtVal.length()) {
					std::stringstream ss;
					ss << newTxtVal;
					ss << ch;
					newTxtVal = ss.str();
				}
			}
		}//switch
		return beingEdited;
	}//acceptInput

	//draw this component on the screen
	void MyUITextBox::draw() {
		float lblX, lblY;
		//draw label offset
		int labelLen = label.length();
		lblX = (x < w ? w : labelLen * 14 * -1);   //check if close to left edge
		lblY = h / 8.0f;
		MyUIComponent::drawBaseCmp(lblX, lblY, lineWt);
		//draw text inside box
		if (beingEdited) {
			std::stringstream ss;
			ss << newTxtVal;
			blinkCount++;
			int maxBCnt = 15;
			if ((maxBCnt - 1) == (blinkCount % maxBCnt)) { blinkCount = 0;         cursorOn = !cursorOn; }
			if (cursorOn) { ss << "|"; }         //blink pipe to act as cursor
			this->drawText(0, 0, ss.str(), true, .2f);
		}
		else {
			this->drawText(0, 0, txtVal, true, .2f);
		}
	}//draw
	//handles ui event
	void MyUITextBox::getValueForEvent(int hand, int type, int drag, float& val) {}
	//handles ui event
	int MyUITextBox::click(float _x, float _y, int cmpObj) {
		if (this->isInside(_x, _y)) {
			beingEdited = true;
			cursorOn = true;
			blinkCount = 0;
			newTxtVal = std::string(txtVal);             //make copy of current text in textbox
		}
		else {
			beingEdited = false;
			cursorOn = false;
			blinkCount = 0;
			newTxtVal = "";
		}
		return ID;
	}
	std::ostream& operator<<(std::ostream& out, const MyUITextBox& cmp) {
		out << static_cast<const MyUIComponent&>(cmp);                                                         //start with base class
		out << "\tText box : Current text : " << cmp.txtVal << "\n";
		out << "\t Max Length : " << cmp.maxChars << " | Current state : " << cmp.tbState << " | Currently being edited : " << cmp.beingEdited << "\n";
		out << "\t Current edit text : '" << cmp.newTxtVal << "'" << "\n";
		return out;
	}//op<<

	int MyUIButton::reInit() {			//any external-specific functionality
		MyUIComponent::reInit();		//parent reinit/init
		init();
		return 0;
	}
	void MyUIButton::init() {}

	void MyUIButton::drawCheckBox() {
		glPushMatrix();
		if (isChecked) { glColor4f(.5f - clr[0] / 2.0f, .5f - clr[1] / 2.0f, .5f - clr[2] / 2.0f, clr[3]); }
		else { glColor4f(clr[0] / 2.0f, clr[1] / 2.0f, clr[2] / 2.0f, clr[3]); }

		glTranslatef(x, y, 0);
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINES);
		glLineWidth(1.0f);
		glBegin(GL_QUADS);
		glVertex2f(0, 0);
		glVertex2f(w, 0);
		glVertex2f(w, h);
		glVertex2f(0, h);
		glEnd();

		if (isChecked) { glColor4f(1.0f - clr[0], 1.0f - clr[1], 1.0f - clr[2], clr[3]); }
		else { glColor4f(clr[0], clr[1], clr[2], clr[3]); }
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glBegin(GL_QUADS);
		glVertex2f(2, 2);
		glVertex2f(w - 4, 2);
		glVertex2f(w - 4, h - 4);
		glVertex2f(2, h - 4);
		glEnd();
		glPopMatrix();
	}//

	//draw this component on the screen
	void MyUIButton::draw() {
		if (isCheckBox) {
			drawCheckBox();
			//draw label?
		}
		else {
			MyUIComponent::drawBaseCmp(0, 0, lineWt);
		}
	}//draw

	//handles kinect event
	void MyUIButton::getValueForEvent(int hand, int type, int drag, float& val) {}
	//handles kinect event
	int MyUIButton::click(float _x, float _y, int cmpObj) {
		if (isCheckBox) { toggleChecked(); }
		return ID;
	}
	std::ostream& operator<<(std::ostream& out, const MyUIButton& cmp) {
		out << static_cast<const MyUIComponent&>(cmp);                                                      //start with base class
		return out;
	}//op<<

	int MyUICaption::reInit() {			//any external-specific functionality
		MyUIComponent::reInit();		//parent reinit/init
		init();
		return 0;
	}
	void MyUICaption::init() {
		flags[UIobjIDX_CanDrag] = false;
		flags[UIobjIDX_CanClick] = false;
	}

	//draw this component on the screen
	void MyUICaption::draw() {
		MyUIComponent::drawText(0, 0, label,lineWt);
	}//draw

	//handles kinect event
	void MyUICaption::getValueForEvent(int hand, int type, int drag, float& val) {}

	//handles kinect event
	int MyUICaption::click(float _x, float _y, int cmpObj) { return ID; }

	std::ostream& operator<<(std::ostream& out, const MyUICaption& cmp) {
		out << static_cast<const MyUIComponent&>(cmp);                                                         //start with base class

		return out;
	}//op<<

}//namespace