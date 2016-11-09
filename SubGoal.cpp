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
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <iostream>

#include "SubGoal.h"

namespace cPBPropApp {


	void SubGoal::buildUI(int idx, int i, string label, int type, vector<float>& vals, vector<int>& objType, vector<string>& objLabels, vector<vector<int>>& objXY_WH, vector<vector<float>>& objClr, vector<float>& dims) {
		stringstream ss;
		string sldrNames[3] = { string("Weight : "), string("Target Val : "), string("Mult : ") };			//main named buttons
		//dims[0] = sliderLen;
		//dims[1] = sliderWide;
		//dims[2] = widBuf;
		//dims[3] = capWide;
		//dims[4] = col2;
		//dims[5] = hWidBuf;
		//dims[6] = qWidBuf;
		//dims[7] = _stOff;
		//dims[8] = capBuf;
		//dims[10] = capYBuff;

		//TODO this is not correct!
	
		float widBufMult = dims[2] * i;
		ss.str("");		ss << "Enable";
		vector<float> vb = { 0, widBufMult, dims[0], dims[0], .75f, .75f, .75f, 1 };
		UI->initUIObj(idx++, ss.str(), 0, vb, objType, objLabels, objXY_WH, objClr);
		ss.str("");		ss << name << " cost : ";
		vector<float> vCap = { dims[5], dims[7] + widBufMult, dims[3], dims[3], .25f, .25f, .25f, 1 };
		UI->initUIObj(idx++, ss.str(), 2, vCap, objType, objLabels, objXY_WH, objClr);

		for (int j = 0; j < 3; ++j) {//3 sliders weight/tar val/mult
			float multX = j * dims[4];
			vector<float> v1 = { (multX)+dims[1], (widBufMult + dims[0]), dims[1], dims[0], .85f, .85f, .85f, 1 };		//sliderwide is width of thumb
			UI->initUIObj(idx++, "", 1, v1, objType, objLabels, objXY_WH, objClr);		//puts appropriate values in array
			ss.str("");		ss << sldrNames[j];
			vector<float> v2 = { (multX)+dims[1] + dims[1], (widBufMult + dims[10]) - dims[8], dims[3], dims[3], .25f, .25f, .25f, 1 };
			UI->initUIObj(idx++, ss.str(), 2, v2, objType, objLabels, objXY_WH, objClr);
		}
	}








}//namespace cPBPropApp