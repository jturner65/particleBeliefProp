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
#ifndef APPS_PARTICLEBELIEFPROP_SubGoal_H_
#define APPS_PARTICLEBELIEFPROP_SubGoal_H_

#include <vector>
#include <string>
#include <Eigen/Dense>
#include <memory>

#include "MyGuiHandler.h"

namespace cPBPropApp {
		///base class to hold the relevant components of a single subgoal
		class SubGoal {
		public:
			typedef double(SubGoal::*CostCalcFunc)(double);		//function pointer typedef to cost function - takes double, returns double
			CostCalcFunc costFunc;								//pointer to cost function
			
			SubGoal(const std::string& _name, double _mu, double _sig, double _wt) : name(_name), active(false), useForVar(false), mean(_mu), std(_sig), wt(_wt),UI(nullptr) {}
			virtual ~SubGoal() {}

			//init from xml
			inline void initSubGoal(double _mu, double _sd, bool _active, bool _useForVar, double _initWt, const std::string& _name, bool sqCost) {
				mean = _mu;		std = _sd;		active = _active;	useForVar = _useForVar;		name = std::string(_name);		wt = _initWt;
				costFunc = (sqCost ? &SubGoal::getSqCostCalc : &SubGoal::getCostCalc);//assign function pointer
			}

			//weight handling
			//set/reset weights
			inline void setWeight(double newWt) { wt = newWt; wtDivStd = wt / std; wtDivSqStd = wtDivStd / std; }
			inline double getWeight(bool reDist) { return (active ? (reDist ? 1 : wt) : 0); }

			void buildUI(int idx, int i, std::string label, int type, std::vector<float>& vals, std::vector<int>& objType, std::vector<std::string>& objLabels, std::vector<std::vector<int>>& objXY_WH, std::vector<std::vector<float>>& objClr, std::vector<float>& dims);
			void setActive(bool val) { active = val; }

			//from/to UI
			void setFromUIVals(double _wt, double _mu, double _invstd) { wt = _wt, mean = _mu, std = 1.0 / _invstd; }
			std::vector<double> accumulateToUIVals() {//wt, mean, 1/std
				std::vector<double> res;
				res.push_back(wt); res.push_back(mean); res.push_back(1.0 / std);
				return res;
			}
			
			//cost calcs
			inline double getCost(double val) { if (!active) { return 0; }return (this->*costFunc)(val); }			
			inline double getSqCostCalc(double val) {double diff = (val - mean);return checkNan(wtDivSqStd * diff * diff, val);}
			inline double getCostCalc(double val) { return  checkNan(wtDivStd * abs(val - mean), val); }
			inline double checkNan(double cost, double val) {
				if (isnan(cost)) { std::cout << "ALERT : Nan cost for subgoal " << name << " with value : " << val << "\n"; cost = 100000000.0; }
				return cost;
			}//checkNan

			friend std::ostream& operator<<(std::ostream& out, SubGoal& subg) {
				out << "Active ? : " << (subg.active ? "True " : "False") << "|\tWeight : " << subg.wt << "\tMean : " << subg.mean << "\tStd : " << subg.std 
					<< "\tSubgoal name : " << subg.name << "\tPreCalced Wt/SD : " << subg.wtDivStd << "\tPreCalced Wt/(SD*SD) : " << subg.wtDivSqStd << "\tUsed for variance calc : " << (subg.useForVar ? "True " : "False") << "\n";
			}

			//variables
		public:
			double wtDivStd,							//precalced wts divided by sds
				wtDivSqStd,								//precalced wts divided by squared sds
				wt,									//weights for subgoal -> should be between 0 and 1
				mean,									//tar mean for subgoal
				std;									//tar std for subgoal
			bool active;									//enable/disable various subgoals 
			bool useForVar;									//whether or not this subgoal is used in variance caluclation

			std::string name;								//name of subgoal weights, for ui;
			std::shared_ptr<MyGuiHandler> UI;				//UI to enable modification of sim variables used by this subgoal


		};//class subgoal

}//namespace cPBPropApp

#endif