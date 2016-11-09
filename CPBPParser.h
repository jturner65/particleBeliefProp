#ifndef CPBPParser_H
#define CPBPParser_H

#include <cstddef>

#include <Eigen/StdVector>
#include <memory>
#include <Eigen/Dense>
#include "CPBPParams.h"
// TinyXML-2 Library
// http://www.grinninglizard.com/tinyxml2/index.html
#include <tinyxml2.h>
#include "dart/utils/Parser.h"
//#include <boost/algorithm/string.hpp>

namespace cPBPropApp {
	/// CPBPParser
	class CPBPParser {
	public:
		/// read configuration data for Control-PBP app
		static void readCPBPConfig(const std::string& _filename, const std::string& dataPath, std::shared_ptr<CPBPParams>& cp, std::vector<size_t>& genCoordIds, Eigen::Ref<Eigen::VectorXd> initConfig);
		/// read params config file
		static void readCPBPParams(const std::string& dataPath, tinyxml2::XMLElement* paramsElement, std::shared_ptr<CPBPParams>& cp);
		static void readParamsFromFilename(tinyxml2::XMLElement* paramsConfig, const std::string& cfgFileName, std::shared_ptr<CPBPParams>& cp);
		/// read skel config file
		static void readCPBPSkelFile(const std::string& dataPath, tinyxml2::XMLElement* skelElement, std::vector<size_t>& genCoordIds, Eigen::Ref<Eigen::VectorXd> initConfig, std::shared_ptr<CPBPParams>& cp);
		/// read UI config file
		static void readCPBPUIFile(const std::string& dataPath, tinyxml2::XMLElement* UIElement);
		
	private:
		static bool lclGetValueBool(tinyxml2::XMLElement* _parentElement, const std::string& _name);

	};
} // namespace cPBPropApp

#endif // #ifndef CPBPParser_H
