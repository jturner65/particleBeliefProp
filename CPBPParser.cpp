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

#include <algorithm>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <iostream>

#include "CPBPParser.h"
#include <boost/lexical_cast.hpp>

using namespace dart::utils;

namespace cPBPropApp {

	void CPBPParser::readCPBPConfig(const std::string& _filename, const std::string& dataPath, std::shared_ptr<CPBPParams>& cp, std::vector<size_t>& genCoordIds, Eigen::Ref<Eigen::VectorXd> initConfig) {
		//default values incase no xml can be found
		genCoordIds.push_back(1);	// root rot around y axis
		genCoordIds.push_back(4);	// root location in y
		genCoordIds.push_back(6);   // left hip
		genCoordIds.push_back(9);   // left knee
		genCoordIds.push_back(10);  // left ankle
		genCoordIds.push_back(13);  // right hip
		genCoordIds.push_back(16);  // right knee
		genCoordIds.push_back(17);  // right ankle
		//genCoordIds.push_back(21);  // lower back
		initConfig.setZero();
		initConfig.head(genCoordIds.size()) << -0.2, 0.05, 0.15, -0.4, 0.25, 0.15, -0.4, 0.25;// , 0.0;
		// Load xml and create Document
		tinyxml2::XMLDocument _configFile;
		try {			dart::utils::openXMLFile(_configFile, _filename.c_str());		}
		catch (std::exception const& e) {			std::cout << "LoadFile  " << _filename << " Fails: " << e.what() << std::endl;		}
		//base document element CPBPConfig
		// skeleton data
		tinyxml2::XMLElement* configElement = NULL;
		configElement = _configFile.FirstChildElement("CPBPConfig");
		if (configElement == NULL) { std::cout << "Config file " << _filename << " does not contain Required <CPBPConfig> as an element.\n";			return;		}

		// skeleton data
		tinyxml2::XMLElement* skelElement = NULL;
		skelElement = configElement->FirstChildElement("skeleton");
		if (skelElement == NULL) { std::cout << "Config file " << _filename << " does not contain <skeleton> as an element.\n";	return;		}
		else {
			std::vector<size_t> tmpCoordIds;
			Eigen::VectorXd tmpInitConfig;
			tmpCoordIds.resize(genCoordIds.size());
			tmpInitConfig.setZero(100);
			for (int i = 0; i < tmpCoordIds.size(); ++i) {
				tmpCoordIds[i] = genCoordIds[i];
				tmpInitConfig(i) = initConfig(i);
			}
			readCPBPSkelFile(dataPath,skelElement, tmpCoordIds, tmpInitConfig, cp);
			genCoordIds.resize(tmpCoordIds.size());
			initConfig.setZero();
			for (int i = 0; i < tmpCoordIds.size(); ++i) {
				genCoordIds[i] = tmpCoordIds[i];
				initConfig(i) = tmpInitConfig(i);
			}
		}

		//params data
		tinyxml2::XMLElement* paramsElement = NULL;
		paramsElement = configElement->FirstChildElement("params");
		if (paramsElement == NULL) { std::cout << "Config file " << _filename << " does not contain Required <params> as an element.\n";		}
		else {			readCPBPParams(dataPath,paramsElement, cp);		}
		//ui data
		tinyxml2::XMLElement* UIconfigElement = NULL;
		UIconfigElement = configElement->FirstChildElement("uiConfig");
		if (UIconfigElement == NULL) { std::cout << "Config file " << _filename << " does not contain Required <uiConfig> as an element.\n";		}
		else {			readCPBPUIFile(dataPath,UIconfigElement);		}

	}//readCPBPConfig

	bool CPBPParser::lclGetValueBool(tinyxml2::XMLElement* _parentElement, const std::string& _name) {
		assert(_parentElement != nullptr);
		assert(!_name.empty());
		std::string str = _parentElement->FirstChildElement(_name.c_str())->GetText();
		if (boost::to_upper_copy(str) == "TRUE" || str == "1") { return true; }
		else if (boost::to_upper_copy(str) == "FALSE" || str == "0") { return false; }
		else { std::cerr << "String Value -->" << str << "<-- does not translate to a valid boolean. Returning false.\n";	return false; }
	}

	void CPBPParser::readParamsFromFilename(tinyxml2::XMLElement* paramsConfig, const std::string& cfgFileName, std::shared_ptr<CPBPParams>& cp) {
		tinyxml2::XMLDocument _prmsCfgFile;
		try { dart::utils::openXMLFile(_prmsCfgFile, cfgFileName.c_str()); }
		catch (std::exception const& e) { std::cout << "Params Config LoadFile  " << cfgFileName << " Fails: " << e.what() << std::endl;				return; }
		std::string pname;
		// individual params in base config data
		tinyxml2::XMLElement* paramElem = NULL;
		paramElem = _prmsCfgFile.FirstChildElement("CPBP_ParamsConfig");
		if (paramElem == NULL) { std::cout << "Params Config file " << cfgFileName << " does not contain <CPBP_ParamsConfig> as an element.\n";				return; }
		else {//read in configuration params
			ElementEnumerator parameters(paramElem, "parameter");
			while (parameters.next()) {
				tinyxml2::XMLElement* pElem = parameters.get();
				assert(pElem != NULL);
				pname = getValueString(pElem, "name");
				if (pname.compare("stateBodiesIDXs") == 0) {//list of idxs denoting RBI bodies = length must == numStateBodies
					Eigen::VectorXd bodyIdxs = getValueVectorXd(pElem, "val");
					cp->setStateRBI(bodyIdxs);
				}
				else {
					std::string v = "val", s = pElem->FirstChildElement(v.c_str())->GetText();
					cp->setParamValFromXMLStr(pname, s);
				}
			}
			cp->setCurrentValsAsDefault();			//initializes important values and sets defaults
		}
	}//readParamsFromFilename

	//read simulation parameters file
	void CPBPParser::readCPBPParams(const std::string& dataPath, tinyxml2::XMLElement* paramsElement, std::shared_ptr<CPBPParams>& cp) {
		//<paramsFile base_filename = "../apps/particleBeliefProp/BaseCPBP_ParamsConfig.xml"
		//	cust_filename = "../apps/particleBeliefProp/CPBP_ParamsConfig.xml" / >
		//load in base first, and then if any cust file is defined, load whatever values are specified within
		std::string pCfgFileName;
		std::stringstream ss;
		tinyxml2::XMLElement* paramsConfig = NULL;
		paramsConfig = paramsElement->FirstChildElement("paramsFile");
		if (paramsConfig == NULL) { std::cout << "Params Config element does not contain <paramsFile> as an element.\n"; }
		else {
			//base config, will set all values
			ss.str("");
			ss << dataPath << getAttribute(paramsConfig, "base_filename");
			pCfgFileName = ss.str();			
			// individual params in base config data - should set all parameters
			readParamsFromFilename(paramsConfig, pCfgFileName, cp);

			//customized config - may only have certain values modified hence we load defaults first above
			ss.str("");
			ss << dataPath << getAttribute(paramsConfig, "cust_filename");
			pCfgFileName = ss.str();
			// individual params in cust config data - will set only those specified
			readParamsFromFilename(paramsConfig, pCfgFileName, cp);

		}

	}//readCPBPParams

	// read skel config file and set initial dof config for skel
	void CPBPParser::readCPBPSkelFile(const std::string& dataPath, tinyxml2::XMLElement* skelElement, std::vector<size_t>& genCoordIds, Eigen::Ref<Eigen::VectorXd> initConfig, std::shared_ptr<CPBPParams>& cp) {
		/**
		eventually handle setting different skeleton file names in skelFile element
				<skelFile filename="/skel/fullbodyPBP.skel" type="humanoid"/>
		<idxInWorld>1</idxInWorld>
		<initConfig filename="../apps/particleBeliefProp/CPBP_InitSkelConfig.xml" />
		*/
		// Load xml and create Document
		std::string skelConfigFile, skelType,strIDXInWorld = "1";
		//int idxInWorld = 1;
		std::stringstream ss;

		tinyxml2::XMLElement* skelFileElem = NULL;
		skelFileElem = skelElement->FirstChildElement("skelFile");
		//skel file elem needs to be of format : <skelFile filename= "{.skel file name}" type="{understood skeleton type (i.e. humanoid)}"></skelFile>
		if (skelFileElem == NULL) { 
			ss << dataPath << "../apps/particleBeliefProp/fullbodyPBP.skel";
			std::cout << "SkelFile element does not contain <skelFile> as an element, using defaults : filename = "<<ss.str()<<" and skelType = humanoid\n";
			//cp->setParamValFromXML("skelFileName", -1, -1, false, ss.str());
			//cp->setParamValFromXML("skelType", -1, -1, false, std::string("humanoid"));
			cp->setParamValFromXMLStr("skelFileName", ss.str());
			cp->setParamValFromXMLStr("skelType", std::string("humanoid"));
		}
		else {
			ss.str("");
			ss << dataPath << getAttribute(skelFileElem, "filename");			//this is skeleton we're going to use, including path relative to DART data path
			skelType = getAttribute(skelFileElem, "type");
			//cp->setParamValFromXML("skelFileName", -1, -1, false, ss.str());
			//cp->setParamValFromXML("skelType", -1, -1, false, skelType);
			cp->setParamValFromXMLStr("skelFileName", ss.str());
			cp->setParamValFromXMLStr("skelType", skelType);
		}

		tinyxml2::XMLElement* skelIdxInWrld = NULL;
		skelIdxInWrld = skelElement->FirstChildElement("idxInWorld");
		if (skelIdxInWrld == NULL) { 
			std::cout << "SkelIdxInWrld element does not contain <idxInWorld> as an element, using defaults idxInWorld of skeleton == 1\n";
		}
		else {
			//idxInWorld = getValueInt(skelElement, "idxInWorld");
			strIDXInWorld = skelIdxInWrld->GetText();
		}
		//cp->setParamValFromXML("idxInWorld", idxInWorld, idxInWorld, false, std::string(""));
		cp->setParamValFromXMLStr("idxInWorld", strIDXInWorld);

		tinyxml2::XMLElement* skelConfig = NULL;
		skelConfig = skelElement->FirstChildElement("initConfig");
		if (skelConfig == NULL) { std::cout << "Skel Config element does not contain <initConfig> as an element, which holds skeleton initial configuration file name.\n";	return; }
		else {					
			ss.str("");
			ss << dataPath << getAttribute(skelConfig, "filename");
			skelConfigFile = ss.str();
			tinyxml2::XMLDocument _skelConfigFile;
			try {				dart::utils::openXMLFile(_skelConfigFile, skelConfigFile.c_str());			}
			catch (std::exception const& e) {
				std::cout << "LoadFile  Skeleton Init Config " << skelConfigFile << " Fails: " << e.what() << std::endl;
				return;
			}
			// skeleton config data
			tinyxml2::XMLElement* configElement = NULL;
			configElement = _skelConfigFile.FirstChildElement("CPBP_InitSkelConfig");
			if (configElement == NULL) { std::cout << "Config file " << skelConfigFile << " does not contain <CPBP_InitSkelConfig> as an element.\n";						}
			else {//read in configuration params
				int numCnfgDofs = 0;
				tinyxml2::XMLElement* configDofs = configElement->FirstChildElement("numConfigDofs");
				if (configDofs != NULL) {
					std::string strNumDofs = configDofs->GetText();
					numCnfgDofs = toInt(strNumDofs);
				}
				else {					return;				}
				int curIdx = 0;
				ElementEnumerator dofConfigs(configElement, "dofConfig");
				genCoordIds.resize(numCnfgDofs);
				while (dofConfigs.next()) {//dofConfigs.get()
					tinyxml2::XMLElement* dfCfg = dofConfigs.get();
					assert(dfCfg != NULL);
					genCoordIds[curIdx] = getValueInt(dfCfg, "idx" );
					initConfig[curIdx] = getValueDouble(dfCfg, "val");
					curIdx++;
				}
				//clear out old values if this is fewer than 9/whatever was initialized into list
			}
		}	
	}//readCPBPSkelFile	
	//configure 
	void CPBPParser::readCPBPUIFile(const std::string& dataPath, tinyxml2::XMLElement* UIElement) {//TODO set means, stds of subgoals	
	
	}//readCPBPUIFile


}  // namespace cPBPropApp
