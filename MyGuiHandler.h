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
#ifndef APPS_PARTICLEBELIEFPROP_MYGUIHANDLER_H_
#define APPS_PARTICLEBELIEFPROP_MYGUIHANDLER_H_

#include <string>
#include <vector>
#include <sstream>
#include "dart/dart.h"

using namespace std;
namespace cPBPropApp {

	class MyUICaption;
	class MyUITextBox;
	class MyUIButton;
	class MyUISlider;

	const int MAX_BUF = 256;

	class MyUIComponent {
		//derived from Kinect UI component library 
	public:
		MyUIComponent() :ID(++ID_gen), flags(0), x(0), y(0), w(0), h(0), clkX(0), clkY(0), stickX(0), stickY(0), UIvalue(0), label(""), msOverTxt(""), stsTxt(""), clr(4), onclick(""), srcOfEvnt(-1), lineWt(1.1f){ init(); }
		MyUIComponent(int _x, int _y, int _w = 0, int _h = 0, float _sX = 0, float _sY = 0, string _lbl = "", string _msOverTxt = "", string _stsTxt = "") :
			ID(++ID_gen),flags(0),x(_x), y(_y), w(_w), h(_h),clkX(0), clkY(0),stickX(_sX), stickY(_sY), UIvalue(0),	label(_lbl), msOverTxt(_msOverTxt), stsTxt(_stsTxt), clr(4), onclick(""), srcOfEvnt(-1), lineWt(1.1f) 
		{
			init();
		}

		MyUIComponent(const MyUIComponent& _mB) : ID(_mB.ID), flags(_mB.flags), x(_mB.x), y(_mB.y), w(_mB.w), h(_mB.h), stickX(_mB.stickX), stickY(_mB.stickY), UIvalue(_mB.UIvalue),
			label(_mB.label), msOverTxt(_mB.msOverTxt), stsTxt(_mB.stsTxt), clr(_mB.clr), onclick(_mB.onclick), srcOfEvnt(_mB.srcOfEvnt), lineWt(_mB.lineWt) {}
		virtual ~MyUIComponent(void) {}

		virtual int reInit();																									//specifically designed to be called manually
		virtual void draw() { glDisable(GL_LIGHTING);	this->draw();	glEnable(GL_LIGHTING); }
		void drawBaseCmp(float delX, float delY, float lineW);           //delx, dely : displacement for label from thumb, line width for text

		//passed offset, text, and (optional) scale amount (to keep text scaled to specific value
		void drawText(float delX, float delY, string text, float lineW, bool manScale = false, float scH = 1);
		void drawTextAtLocation(float x, float y, string text, float lineW, bool manScale, float scH);

		virtual int click(float _x, float _y, int cmpObj);
		virtual int drag(float _x, float _y);

		//returns value of control based on event - l/r hand, click down, up or drag - overridden in child classes  TODO
		virtual void getValueForEvent(int hand, int type, int drag, float& val) {}
		virtual float getCurValue() { return 0; }

		bool checkFlags(int idx) { return this->flags[idx]; }

		//check if this object is currently being dragged by the passed controller (either mouse, left hand or right hand)
		bool checkDragByThisCntrl(int cntrl) { return (this->flags[UIobjIDX_MSDrag] && (cntrl == this->srcOfEvnt)); }
		bool canDrag() { return this->flags[UIobjIDX_CanDrag]; }

		void setOnClick(string& oc) { onclick = oc; }
		string getOnClick() { return onclick; }

		//configuration values
		void setLoc(float _x, float _y) {
			x = _x; y = _y;
			//buildHotSpot();
		}
		void setDim(float _w, float _h) {
			w = _w; h = _h;
			//buildHotSpot();
		}

		void setFlags(int idx, bool val) { flags[idx] = val; }
		void setColor(float r, float g, float b, float a) { clr[0] = r; clr[1] = g; clr[2] = b; clr[3] = a; }
		void setStick(float _x, float _y) { stickX = _x; stickY = _y; }
		void setLabel(string _txt) { label = _txt; }
		void setMSOverTxt(string _txt) { msOverTxt = _txt; }
		void setStsTxt(string _txt) { stsTxt = _txt; }
		void setUIProps(bool _canClick, bool _canDrag) { flags[UIobjIDX_CanClick] = _canClick; flags[UIobjIDX_CanDrag] = _canDrag; }

		void setShown(bool show) {
			flags[MyUIComponent::UIobjIDX_Display] = (flags[MyUIComponent::UIobjIDX_NeverHide] ? true : show);
			if (!flags[MyUIComponent::UIobjIDX_Display]) {		//if being hidden then reset all UI -related stuff
				flags[MyUIComponent::UIobjIDX_MSDown] = false;
				clearClicked(-1, -1, srcOfEvnt);
			}
		}

		void setSrcOfEvnt(int _src) { srcOfEvnt = _src; }

		int getSrcOfEvnt() { return srcOfEvnt; }
		std::string getLabel() { return label; }
		std::string getMouseOverTxt() { return ("" == msOverTxt ? label : msOverTxt); }
		std::string getStatusTxt() { return ("" == stsTxt ? label : stsTxt); }
		int getID() { return ID; }

		virtual string buildLabel(bool showColon, MyUIComponent* cmp) { return this->buildLabel(showColon, cmp); }                                                                             //modify labels on the fly

		virtual bool isClicked() { return flags[UIobjIDX_MSDown]; }
		virtual void clearObjClicked(float _x, float _y, int _srcOfEvnt) {}                                                          //clear object-specific code

		void clearClicked(float _x, float _y, int _srcOfEvnt) {
			this->clearObjClicked(_x, _y, _srcOfEvnt);                                                                            //call object-specific code
			if (_srcOfEvnt == this->srcOfEvnt) {                                                                                      //if this object caused click, then clear click
				if (flags[UIobjIDX_MSDown]) { flags[UIobjIDX_Click] = true; }														//if mouse down happened on this object, set flag that it has been clicked
				else { flags[UIobjIDX_Click] = false; }																				//otherwise clear that it has been clicked - mse down happened elsewhere or not yet
				flags[UIobjIDX_MSDown] = false;    flags[UIobjIDX_MSDrag] = false;
				stickX = -1;   stickY = -1; this->srcOfEvnt = -1;
			}
		}//clearClicked

		//set upon initial click in an object
		//_x,_y location of click
		//cmpobj - component object clicked on (when objects are compound objects like sliders or crank bars, this is what part was clicked in) 0 : not clicked in, 1 : button part, 2 : sliderbar/crank bar part
		//_srcOfEvnt - whether this was from the mouse or the left or right hands
		void setClicked(float _x, float _y, int cmpObj, int _srcOfEvnt) {
			if (flags[UIobjIDX_CanClick]) {
				flags[UIobjIDX_MSDown] = true;
				this->srcOfEvnt = _srcOfEvnt;
				if (flags[UIobjIDX_CanDrag]) { flags[UIobjIDX_MSDrag] = true; }                                              //if this can be dragged, and it isn't already dragging, then set dragged to passed bool                        
				MyUIComponent::click(_x, _y, cmpObj);	                                                                        //object specific click down handling
			}
		}//setClicked

		//return -1 if fail
		int setDragged(bool _drag, float _x, float _y, int _srcOfDrag) {
			if ((flags[UIobjIDX_CanDrag]) && (flags[UIobjIDX_MSDrag]) && (_srcOfDrag == this->srcOfEvnt)) {				        //if it can drag and it is dragging by querying event source, then drag
				return MyUIComponent::drag(_x, _y);		                                                                        //id of object being dragged if success
			}
			return -1;
		}
		//set whether or not this object should be displayed
		void setDisplay(bool _disp) { flags[UIobjIDX_Display] = _disp; }
		//stick to center of object - put center values into _hx and _hy
		virtual void stickToClick(int& hndX, int& hndY) {
			hndX = stickX + x;
			hndY = stickY + y;
		}

		//set appropriate "stick to" spot for hands, relative to x,y vals of UI obj - should snap and stick to this location on click and drag - override for crank, to snap to center of bar
		virtual void stickHandSpot() {
			stickX = (w / 2);
			stickY = (h / 2);
		}

		//force hand to specific spot within ui object, for drag functionality
		void stickHandToClick(int& hndX, int& hndY) {
			this->stickHandSpot();
			this->stickToClick(hndX, hndY);
		}

		void setWinSize(int _wx, int _wy) { winX = _wx; winY = _wy; }
		//check if query location is inside control's boundaries - returns 0 or 1 by default, will return other values if inside parts of multipart ui constructs		
		virtual int isInside(int clckX, int clckY) { return (((clckX > x) && (clckX < x + w) && (clckY > y) && (clckY < y + h)) && (flags[UIobjIDX_Display]) && (flags[UIobjIDX_CanClick]) ? 1 : 0); }

		////copyswap
		//MyUIComponent& operator=(MyUIComponent other) {		//other is already tmp copy
		//	swap(static_cast<MyUIComponent>(*this), static_cast<MyUIComponent>(other));
		//	return *this;
		//}

		//friend void swap(MyUIComponent& _a, MyUIComponent& _b){
		//	using std::swap;
		//	swap(_a.ID, _b.ID);
		//	swap(_a.winX, _b.winX);
		//	swap(_a.winY, _b.winY);
		//	swap(_a.clkX, _b.clkX);
		//	swap(_a.clkY, _b.clkY);
		//	swap(_a.x, _b.x);
		//	swap(_a.y, _b.y);
		//	swap(_a.w, _b.w);
		//	swap(_a.h, _b.h);
		//	//swap(_a.hsXL, _b.hsXL);
		//	//swap(_a.hsXH, _b.hsXH);
		//	//swap(_a.hsYL, _b.hsYL);
		//	//swap(_a.hsYH, _b.hsYH);
		//	swap(_a.stickX, _b.stickX);
		//	swap(_a.stickY, _b.stickY);
		//	swap(_a.UIvalue, _b.UIvalue);
		//	swap(_a.label, _b.label);
		//	swap(_a.msOverTxt, _b.msOverTxt);
		//	swap(_a.stsTxt, _b.stsTxt);
		//	swap(_a.clr, _b.clr);
		//	swap(_a.onclick, _b.onclick);
		//	swap(_a.srcOfEvnt, _b.srcOfEvnt);
		//}

		friend std::ostream& operator<<(std::ostream& out, const MyUIComponent& cmp);

	protected:		//functions
		virtual void init();																									//specifically designed to only be called by constructor

		//protected :
	public:	//variables 
		static unsigned int ID_gen;
		int ID;
		int winX, winY;                                 //size of window holding object
		vector<bool> flags;							    //state flags for this object
		float x, y, w, h;								    //x,y location of upper left corner, width and height of actual control - slider button or push button
		float clkX, clkY;							    //last click x,y value
		//float hsXL, hsXH, hsYL, hsYH, hsXBr, hsYBr;	    //hotspots - click area on screen, in screen coords - depends on UI, in mouse click coords -- only needed for kinect
		float stickX, stickY;						    //location of sticking point in control for kinect-driven hands 
		vector<float> UIvalue;						    //value of control when activated, based on event type  TODD
		string label, msOverTxt, stsTxt;                //current label, mouse over popup text, status bar text(each should default to label if empty)
		vector<float> clr;							    //color of object, 0-1 for each value
		string onclick;
		int srcOfEvnt;                                  //source object of currently engaged event - KC_LEFT: left hand, KC_RIGHT : right hand, KC_MOUSE: mouse

		float lineWt;									//how thick the text for this object should be
		//state flags
		//immediate draw flags
		static const int UIobjIDX_Display = 0;						//display the ui object
		static const int UIobjIDX_DispHS = 1;                        //display hotspot around object
		static const int UIobjIDX_DispMO = 2;                        //display mouseover popuptext
		static const int UIobjIDX_OffsLbl = 3;                        //offset label for sliders and cranks
		//capability flags
		static const int UIobjIDX_CanClick = 4;						//object is clickable
		static const int UIobjIDX_CanDrag = 5;						//object is draggable
		static const int UIobjIDX_CanDispMO = 6;                        //mouse over text is enabled for this object
		static const int UIobjIDX_CanType = 7;                        //object accepts/requires specifically keyboard input from user to change data (text box, listbox w/string values (?))
		//immediate action flags
		static const int UIobjIDX_MSDown = 8;						//mouse/hand has pressed on object
		static const int UIobjIDX_MSUp = 9;						//mouse/hand has release object
		static const int UIobjIDX_MSDrag = 10;						//mouse/hand is dragging object					   
		//event related flags
		static const int UIobjIDX_Click = 11;                        //click event recognized
		//debug
		static const int UIobjIDX_DispDebug = 12;                       //display debug info - what interface (mouse, lhand, rhand) caused event
		static const int UIobjIDX_CtrLbl = 13;							//center the label or move it to the edge
		static const int UIobjIDX_NeverHide = 14;						//object is always shown and cannot be hidden

		static const int UIobj_numFlags = 15;

		//for event value return - idx into object's value array - based on event change return value
		static const int UIobj_numVals = 3;
		//sensitivity of slider/crank dragging - should be 0.0 - 1.0
		//border around slider object
		float SldrBrdr = 2;

		float UIobj_DragSens = .75f;                                                             //type:float      cmt:sensitivity of slider bar drag - 0.0-1.0
		float UIobj_CrankSens = .02f;                                                           //type:float      cmt:sensitivity of crank bar drag - 0.0-1.0
		//font display values - min and max scaling value
		const float UIobj_MinFontScale = .08f;                                                        //type:float      cmt:min value for font scaling for ui object labels
		const float UIobj_MaxFontScale = .3f;                                                         //type:float      cmt:max value for font scaling for ui object labels

	};//MyUIComponent


	class MyUISlider : public MyUIComponent {
	public:
		MyUISlider() :MyUIComponent(), slideMin(0), slideRng(0), slidePos(0), slideSubdiv(0), sbDimL(0), sbDimTh(0), horiz(true), LToR(true), useListVals(false), useCaption(false), dispMinMax(false), dispListVals(0), resListVals(0) { init(); }
		MyUISlider(int _x, int _y, int _w = 0, int _h = 0, float _sX = 0, float _sY = 0, string _lbl = "", string _msOverTxt = "", string _stsTxt = "") :
			MyUIComponent(_x, _y, _w, _h, _sX, _sY, _lbl, _msOverTxt, _stsTxt), slideMin(0), slideRng(0), slidePos(0), slideSubdiv(0), sbDimL(0), sbDimTh(0), horiz(true), LToR(true), useListVals(false), dispMinMax(false), useCaption(false), dispListVals(0), resListVals(0) {
			init();
		}							//local init		

		MyUISlider(const MyUISlider& _mB) :
			MyUIComponent(_mB), slideMin(_mB.slideMin), slideRng(_mB.slideRng), slidePos(_mB.slidePos), slideSubdiv(_mB.slideSubdiv), sbDimL(_mB.sbDimL), sbDimTh(_mB.sbDimTh), horiz(_mB.horiz),
			LToR(_mB.LToR), useListVals(_mB.useListVals), dispMinMax(_mB.dispMinMax), dispListVals(_mB.dispListVals), resListVals(_mB.resListVals) {}
		~MyUISlider(void) {}

		virtual int reInit();
		virtual void draw();
		void drawSliderBar();

		void setCaption(std::shared_ptr<MyUICaption> _cap, std::shared_ptr<MyUISlider> _sldr);

		virtual void clearObjClicked(float _x, float _y, int _srcOfEvnt) {}
		virtual int click(float _x, float _y, int cmpObj);
		virtual int drag(float _x, float _y);

		virtual void getValueForEvent(int hand, int type, int drag, float& val);
		virtual float getCurValue() { return slideMin + (LToR ? slidePos : 1 - slidePos) * slideRng; }

		void setSliderVal(float _val) { 
			float sld = ((_val - slideMin) / slideRng); 
			if (sld > 1) { setSlideMax(1.01f*_val); setSliderVal(_val); };
			if (sld < 0) { float max = slideRng - slideMin;  setSlideMin(.95*_val); setSlideMax(max); setSliderVal(_val); };
			setSlidePos((LToR ? sld : 1 - sld));
		}
		
		void setSlideMin(float _sm) { slideMin = _sm; }
		void setSlideMax(float _sMx) { slideRng = _sMx - slideMin; }
		void setSlideRng(float _sl) { slideRng = _sl; }
		void setSlidePos(float _sp) { slidePos = _sp; }
		void setSlideSubdiv(int _ssv) { slideSubdiv = _ssv; }
		void setSbDimW(int _sx) { sbDimL = _sx; }
		void setSbDimH(int _sy) { sbDimTh = _sy; }

		float getSliderPos() { return slidePos; }

		void setSlideVals(float _sl, float _sp, int _ssv, int _sx, int _sy) {
			slideRng = _sl;	slidePos = _sp;	slideSubdiv = _ssv;	sbDimL = _sx; sbDimTh = _sy;
			//	buildHotSpot();
		}

		void setIsHoriz(bool _hz) { horiz = _hz; }
		void setIsLToR(bool _lt) { LToR = _lt; }
		virtual string buildLabel(bool showColon, MyUISlider* cmp) {
			if (cmp->useListVals) {
				return buildLabel(showColon, dispListVals[(int)cmp->getCurValue()]);
			}
			else {
				return buildLabel(showColon, cmp->getCurValue(), "%.4f");
			}
		}
		string buildLabel(bool showColon, float val, const char* fmt) {                                          //build label to include current slider value
			char buf[MAX_BUF];
			std::string::size_type colLoc = label.find_first_of(":");
			stringstream ss;
			sprintf(buf, fmt, val);
			if (showColon) { ss << label.substr(0, colLoc) << ":"; }
			ss << buf;
			return ss.str();// label = ss.str();
		}//buildLabel
		string buildLabel(bool showColon, string& val) {                                          //build label to include current slider value
			std::string::size_type colLoc = label.find_first_of(":");
			stringstream ss;
			if (showColon) { ss << label.substr(0, colLoc) << ":"; }
			ss << val;
			return ss.str();// label = ss.str();
		}//buildLabel
		//check if query location is inside control's hotspot - used with hands to allow them to snap to center of control object
		//virtual int isInsideHotSpot(int clckX, int clckY) { return (((clckX > hsXL) && (clckX < hsXH) && (clckY > hsYL) && (clckY < hsYH)) ? 2 : 0); }

		virtual int isInside(int clckX, int clckY) {
			if ((!flags[UIobjIDX_Display]) || (!flags[UIobjIDX_CanClick])) { return 0; }		//if can't display or click item then alway return 0
			int inThumb = isInsideThumb(clckX, clckY), inTrack = isInsideTrack(clckX, clckY);
			return (inThumb || inTrack) ? (inThumb ? 1 : 2) : 0;
		}//isInside
		//checks if in thumb (button part) of slider
		bool isInsideThumb(int clckX, int clckY) {
			float offset = (horiz ? -(w / 2.0) + slidePos * sbDimL : -(h / 2.0) + (slidePos * sbDimL));					//displacement of thumb based on orientation and value	
			return (horiz ?
					((clckX > x + offset) && (clckX < x + w + offset) && (clckY > y) && (clckY < y + h)) :				//displace "hotspot" in x if horizontal
					((clckX > x) && (clckX < x + w) && (clckY > y + offset) && (clckY < y + h + offset)));				//displace in y if vertical
		}//isInsideThumb

		//checks if in track part of slider
		bool isInsideTrack(int clckX, int clckY) {
			float lenX = (horiz ? sbDimL : sbDimTh),						//dimensions of slider based on orientation
				lenY = (horiz ? sbDimTh : sbDimL),
				trX = x + (horiz ? 0 : (abs(w - sbDimTh) / 2)),			//location of slider bar based on orientation
				trY = y + (horiz ? (abs(h - sbDimTh) / 2) : 0);
			return ((clckX >= trX) && (clckX <= trX + lenX)) && ((clckY >= trY) && (clckY <= trY + lenY));
		}//isInsideTrack

		virtual void stickToClick(int& hndX, int& hndY) {
			float offset = (horiz ? -(w / 2.0) + slidePos * sbDimL : -(h / 2.0) + (slidePos * sbDimL));					//displacement of thumb based on orientation and value
			hndX = stickX + x + int(horiz ? offset : 0);
			hndY = stickY + y + int(horiz ? 0 : offset);
		}

		//MyUISlider& operator=(MyUISlider other) {
		//	swap(static_cast<MyUISlider>(*this), static_cast<MyUISlider>(other));
		//	return *this;
		//}
		//friend void swap(MyUISlider& _a, MyUISlider& _b) {
		//	using std::swap;
		//	swap(static_cast<MyUIComponent>(_a), static_cast<MyUIComponent>(_b));
		//	swap(_a.slideMin, _b.slideMin);
		//	swap(_a.slideRng, _b.slideRng);
		//	swap(_a.slidePos, _b.slidePos);
		//	swap(_a.slideSubdiv, _b.slideSubdiv);
		//	swap(_a.sbDimL, _b.sbDimL);
		//	swap(_a.sbDimTh, _b.sbDimTh);
		//	swap(_a.horiz, _b.horiz);
		//	swap(_a.LToR, _b.LToR);
		//	swap(_a.useListVals, _b.useListVals);
		//	swap(_a.useCaption, _b.useCaption);
		//	swap(_a.dispListVals, _b.dispListVals);
		//	swap(_a.dispMinMax, _b.dispMinMax);
		//	swap(_a.resListVals, _b.resListVals);
		//	swap(_a.labelHolder, _b.labelHolder);
		//}//swap

		friend std::ostream& operator<<(std::ostream& out, const MyUISlider& cmp);

	protected:	//functions
		virtual void init();
		void capSlidePos() { if (slidePos >= 1) { slidePos = 1; } else if (slidePos <= 0) { slidePos = 0; } }				//bound sliderpos values to between 0 and 1

	public :
		bool useListVals;                   //use list values instead of numerics for this slider (like a list box)
		vector<std::string> dispListVals;   //what values are displayed if this is used as a list box
		 
	protected:	//variables					//along with inherited members from component, need to initialize the following
		float slideMin;                     //min value for slider
		float slideRng;						//length of sliding capability == max value (if 0 is min)
		float slidePos;						//0-1 position along slider -> 0 at top/left, 1 at bottom/right : multiplied with slideRng to be used as value of slider
		int slideSubdiv;					//# of subdivisions of slider for discrete values - <= 0 means "continuous"
		int sbDimL, sbDimTh;				//dimensions of sliderbar track (where slider thumb moves) - length and thickness
		bool horiz;							//orientation of slider - true is horizontal, false is vertical
		bool LToR;                          //whether the values INCREASE left to right(horiz)/up to down(vert) or right to left/down to up
		bool useCaption;					//use a caption to show this slider's value instead of displaying it in the slider thumb
		bool dispMinMax;					//display min and max slider vals at ends of slider
		vector<std::string> resListVals;    //what values are returned as results if this is a list box
		std::shared_ptr<MyUICaption> labelHolder;			//caption to display name and value of this slider

	};//MyUISlider

	class MyUITextBox : public MyUIComponent {
	public:
		MyUITextBox() :MyUIComponent(), txtVal(""), newTxtVal(""), tbState(0), maxChars(10), beingEdited(false), cursorOn(false), blinkCount(0) { init(); }
		MyUITextBox(int _x, int _y, int _w = 0, int _h = 0, float _sX = 0, float _sY = 0, string _lbl = "", string _msOverTxt = "", string _stsTxt = "") :
			MyUIComponent(_x, _y, _w, h, _sX, _sY, _lbl, _msOverTxt, _stsTxt), txtVal(""), newTxtVal(""), tbState(0), maxChars(10), beingEdited(false), cursorOn(false), blinkCount(0) {
			init();
		}				//local init

		MyUITextBox(const MyUITextBox& _mB) :MyUIComponent(_mB), txtVal(_mB.txtVal), newTxtVal(_mB.newTxtVal), tbState(_mB.tbState),
			maxChars(_mB.maxChars), beingEdited(_mB.beingEdited), cursorOn(_mB.cursorOn), blinkCount(_mB.blinkCount) {}//should be copyswap
		~MyUITextBox(void) {}

		virtual int reInit();                                                                   //specifically designed to be called manually
		virtual void draw();
		virtual bool isClicked() {
			bool retVal = flags[UIobjIDX_Click];
			flags[UIobjIDX_Click] = false;
			return retVal;
		}        //returns whether this object has been clicked, and then clears click value

		bool isBeingEdited() { return beingEdited; }

		virtual void clearObjClicked(float _x, float _y, int _srcOfEvnt) {
			if (!this->isInside(_x, _y)) {
				beingEdited = false;
				cursorOn = false;
				blinkCount = 0;
				txtVal = ("" == newTxtVal ? txtVal : string(newTxtVal));
				newTxtVal = "";
			}
		}//clear values if clearClicked-generating event happened within this object

		bool acceptInput(unsigned char ch);

		virtual int click(float _x, float _y, int cmpObj);
		virtual int drag(float _x, float _y) { return -1; }

		virtual void getValueForEvent(int hand, int type, int drag, float& val);
		virtual float getCurValue() { return -1; }	                                               //use to pass state? NOT USED FOR TB		
		string getCurTxtVal() { return (beingEdited ? newTxtVal : txtVal); }
		int getMaxchars() { return maxChars; }

		void setMaxChars(int _mx) { maxChars = _mx; }

		virtual string buildLabel(bool showColon, MyUIComponent* cmp) { return label; }                                                           //modify labels on the fly

		//MyUITextBox& operator=(MyUITextBox other) {
		//	swap(static_cast<MyUITextBox>(*this), static_cast<MyUITextBox>(other));
		//	return *this;
		//}

		//friend void swap(MyUITextBox& _a, MyUITextBox& _b) {
		//	using std::swap;
		//	swap(static_cast<MyUIComponent>(_a), static_cast<MyUIComponent>(_b));
		//	swap(_a.txtVal, _b.txtVal);
		//	swap(_a.newTxtVal, _b.newTxtVal);
		//	swap(_a.tbState, _b.tbState);
		//	swap(_a.maxChars, _b.maxChars);
		//	swap(_a.beingEdited, _b.beingEdited);
		//	swap(_a.cursorOn, _b.cursorOn);
		//	swap(_a.blinkCount, _b.blinkCount);
		//}//swap

		friend std::ostream& operator<<(std::ostream& out, const MyUITextBox& cmp);

	private: //functions
		virtual void init();

	protected: //variables
		std::string txtVal, newTxtVal;                                                      //current accepted text input, and current "being added" text input
		int tbState;                                                                        //state of text box - being edited? has been changed (i.e. dirty)?
		int maxChars;                                                                       //max number of input characters == max length of txtVal
		bool beingEdited;                                                                   //this text box is currently accepting input
		bool cursorOn;                                                                      //whether edit cursor is on or off
		int blinkCount;                                                                     //blink counter for cursor - set number of cycles on/off
	};//MyUITextBox
	class MyUIButton : public MyUIComponent {
	public:
		MyUIButton() :MyUIComponent(), isCheckBox(false) { init(); }
		MyUIButton(int _x, int _y, int _w = 0, int _h = 0, float _sX = 0, float _sY = 0, string _lbl = "", string _msOverTxt = "", string _stsTxt = "") :
			MyUIComponent(_x, _y, _w, h, _sX, _sY, _lbl, _msOverTxt, _stsTxt), isCheckBox(false) {
			init();
		}				//local init
		MyUIButton(const MyUIButton& _mB) :MyUIComponent(_mB), isCheckBox(_mB.isCheckBox), isChecked(_mB.isChecked) {}//should be copyswap
		~MyUIButton(void) {}

		virtual int reInit();																									//specifically designed to be called manually
		virtual void draw();
		virtual bool isClicked() { bool retVal = flags[UIobjIDX_Click]; flags[UIobjIDX_Click] = false;    return retVal; }
		virtual void clearObjClicked(float _x, float _y, int _srcOfEvnt) {}
		virtual int click(float _x, float _y, int cmpObj);
		void drawCheckBox();
		virtual int drag(float _x, float _y) { return -1; }
		int toggleChecked() { isChecked = !isChecked; return(isChecked ? 1 : 0); }
		virtual void getValueForEvent(int hand, int type, int drag, float& val);
		virtual float getCurValue() { if (isCheckBox) { return (isChecked ? 1 : 0); } return 1; }							//button will just provide single value with default event unless is checkbox
		virtual string buildLabel(bool showColon, MyUIComponent* cmp) { return label; }                                //modify labels on the fly
		//MyUIButton& operator=(MyUIButton other) {
		//	swap(static_cast<MyUIButton>(*this), static_cast<MyUIButton>(other));
		//	return *this;
		//}

		//friend void swap(MyUIButton& _a, MyUIButton& _b) {
		//	using std::swap;
		//	swap(static_cast<MyUIComponent>(_a), static_cast<MyUIComponent>(_b));
		//	swap(_a.isCheckBox, _b.isCheckBox);
		//	swap(_a.isChecked, _b.isChecked);
		//}//swap

		friend std::ostream& operator<<(std::ostream& out, const MyUIButton& cmp);

	private: //functions
		virtual void init();

	public: //variables
		bool isCheckBox;						//whether this button should be considered a checkbox or not
		bool isChecked;							//if this button is a checkbox, this means the box is checked

	};//MyUIButton

	class MyUICaption : public MyUIComponent {
	public:
		MyUICaption() :MyUIComponent() { init(); }
		MyUICaption(int _x, int _y, int _w = 0, int _h = 0, float _sX = 0, float _sY = 0, string _lbl = "", string _msOverTxt = "", string _stsTxt = "") :
			MyUIComponent(_x, _y, _w, h, _sX, _sY, _lbl, _msOverTxt, _stsTxt) {
			init();
		}				//local init
		MyUICaption(const MyUICaption& _mB) :MyUIComponent(_mB) {}//should be copyswap
		~MyUICaption(void) {} 

		virtual int reInit();																									//specifically designed to be called manually
		virtual void draw();
		virtual bool isClicked() { bool retVal = flags[UIobjIDX_Click]; flags[UIobjIDX_Click] = false;    return retVal; }
		virtual void clearObjClicked(float _x, float _y, int _srcOfEvnt) {}
		virtual int click(float _x, float _y, int cmpObj);
		virtual int drag(float _x, float _y) { return -1; }

		virtual void getValueForEvent(int hand, int type, int drag, float& val);
		virtual float getCurValue() { return 1; }							//button will just provide single value with default event
		string buildLabel(MyUISlider* cmp) {
			if (cmp->useListVals) {
				return buildLabel(true, cmp->dispListVals[(int)cmp->getCurValue()]);
			}
			else {
				return buildLabel(true, cmp->getCurValue(), "%.4f");
			}
		}
		virtual string buildLabel(bool showColon, MyUIComponent* cmp) {
			return buildLabel(showColon, cmp->getCurValue(), "%.4f");
		}                                       //build label to include current slider value
		string buildLabel(bool showColon, float val, const char* fmt) {
			char buf[MAX_BUF];
			std::string::size_type colLoc = label.find_first_of(":");
			stringstream ss;
			sprintf(buf, fmt, val);
			if (showColon) { ss << label.substr(0, colLoc) << ":"; }
			ss << buf;
			return ss.str();// label = ss.str();
		}//buildLabel

		string buildLabel(bool showColon, string str) {
			std::string::size_type colLoc = label.find_first_of(":");
			stringstream ss;
			if (showColon) { ss << label.substr(0, colLoc) << ":"; }
			ss << str;
			return ss.str();// label = ss.str();
		}//buildLabel


		//MyUICaption& operator=(MyUICaption other) {
		//	swap(static_cast<MyUICaption>(*this), static_cast<MyUICaption>(other));
		//	return *this;
		//}

		//friend void swap(MyUICaption& _a, MyUICaption& _b) {
		//	using std::swap;
		//	swap(static_cast<MyUIComponent>(_a), static_cast<MyUIComponent>(_b));
		//	swap(_a.sliderOwner, _b.sliderOwner);
		//}//swap

		friend std::ostream& operator<<(std::ostream& out, const MyUICaption& cmp);

	private: //functions
		virtual void init();

	public: //variables
		std::shared_ptr<MyUISlider> sliderOwner;			//slider that owns this caption

	};//MyUICaption
	
	//handle UI interactions
	class MyGuiHandler {
	public:
		MyGuiHandler(float _stX, float _stY, double _w, double _h);
		virtual ~MyGuiHandler();

		//initialize object vectors for hardcoded UI
		void initUI();
		void initUIObj(int i, string label, int type, vector<float>& vals, vector<int>& objType, vector<string>& objLabels, vector<vector<int>>& objXY_WH, vector<vector<float>>& objClr);
		//build UI - hardcoded values for now
		void setUI(vector<int>& objType, vector<string>& objLabels, vector<vector<int>>& objXY_WH, vector<vector<float>>& objClr);
		void setUI(vector<int>& objType, vector<string>& objLabels, vector<vector<int>>& objXY_WH, vector<vector<float>>& objClr, float barLen);
		//for each type being supported
		MyUIButton* setUIButton(string& label, vector<int>& objXY_WH, vector<float>& objClr);
		//MyUISlider* setUISlider(string& label, vector<int>& objXY_WH, vector<float>& objClr);
		MyUISlider* setUISlider(string& label, vector<int>& objXY_WH, vector<float>& objClr, float _barLen);
		MyUITextBox* setUITextBox(string& label, vector<int>& objXY_WH, vector<float>& objClr);
		MyUICaption* setUICaption(string& label, vector<int>& objXY_WH, vector<float>& objClr);
		void setBaseVals(MyUIComponent* cmp, string& label, vector<int>& objXY_WH, vector<float>& objClr);
		//handle UI element clicked
		//bool handleUIClick();
		//draw debug bounding box around UI
		void drawBoundBox();
		//draw all objects
		void drawObjs();
		//draw single object - called externally - retranslates in global frame.
		void drawObj(int idx);
		//hide or show UI, and reset all objects accordingly
		void setShowUI(bool show);
		//set button to be show-hide button by idx in MyPgUIBtn
		void setShowHideBtn(int idx);

		//go through all elements and check to see if events have occured in them
		//d <= 0 means event triggered by mouse -  depth is for push events with kinect
		//mouseEvent : if this is an event generated by the mouse or by the kinect
		//btn : which button was clicked
		//evnt : the nature of the event : 0 : up, 1 : click/push, 2 : drag/grab
		bool checkUIElements(int& x, int& y, int& d, bool mouseEvent, int btn, int evnt);
		//check individual element from external call
		bool checkUIElement(int idx, int& xp, int& yp, int& d, bool mouseEvent, int btn, int evnt) {
			int x = xp - stX, y = yp - stY;
			bool msDragEvent = (2 == evnt) ? true : false;														
			bool msClickEvent = (1 == evnt) ? true : false;
			bool objFound = false;
			std::shared_ptr<MyUIComponent>cmp = MyPgUICmp[idx];
			int clkVal = (cmp->isInside(x, y));                                                     //value returned from isInside - what part of object click was inside
			if ((0 != clkVal) || (cmp->checkDragByThisCntrl(UI_MOUSE))) {                           //check if inside this object, or if this object is currently being dragged
				if (!msDragEvent) {
					if (msClickEvent) { cmp->setClicked(x, y, clkVal, UI_MOUSE); }
					else { cmp->clearClicked(x, y, UI_MOUSE); }				//upon click up clear click event - needs to happen cycle after "click release" has been processed
				}
				else {
					int resD = cmp->setDragged(msDragEvent, x, y, UI_MOUSE);
					if (resD != -1) { cmp->stickToClick(x, y); }
				}										                                             //set drag state to be whether or not this is a drag event						
				objFound = true;
			}
			else { cmp->clearClicked(x, y, UI_MOUSE); }									             //use location = -1,-1 for click release of object with mouse outside of object (location ignored)
			return objFound;
		}

		int numComponents() { return MyPgUICmp.size(); }
		void addUIButton(MyUIButton* btn);
		void addUISlider(MyUISlider* sld);
		void addUITextBox(MyUITextBox* tbx);
		void addUICaption(MyUICaption* cap);

	public: //variables
		int numObjs;										//# of ui objects
		float stX, stY;										//upper left corner of UI "frame"
		float width, height;								//width and height of this "page" of UI controls
		double winX, winY;									//window size
		std::string name;
		std::string onload;
		int numTypes = 4;									//# of different UI components implemented

		std::shared_ptr<MyUIButton> showHideBtn;			//button that controls whether this page of UI controls is shown or hidden (always shown and click-processed)

		std::shared_ptr<MyUICaption> debugCap;				//caption to display debug/console data on screen

		vector<std::shared_ptr<MyUIComponent> >    MyPgUICmp;
		vector<std::shared_ptr<MyUIButton> >       MyPgUIBtn;
		vector<std::shared_ptr<MyUISlider> >       MyPgUISldr;
		vector<std::shared_ptr<MyUICaption> >		MyPgUICaption;
		vector<std::shared_ptr<MyUITextBox> >		MyPgUITextBox;							
		const int UI_LEFT = 0;
		const int UI_RIGHT = 1;
		const int UI_MOUSE = 2;
		Eigen::Vector3f oldMousePos;

		bool showUI;								//whether UI is shown or not

		static const float sldrBarLen;
	};
}
#endif  // APPS_PARTICLEBELIEFPROP_MYGUIHANDLER_H_