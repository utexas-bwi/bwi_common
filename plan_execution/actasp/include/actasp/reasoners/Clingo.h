#ifndef actasp_Clingo_h__guard
#define actasp_Clingo_h__guard

#include <actasp/reasoners/Clingo4_2.h>
#include <actasp/reasoners/Clingo4_5.h>

namespace actasp {

struct Clingo {

	static FilteringQueryGenerator* getQueryGenerator(const std::string& incrementalVar,
          const std::string& queryDir,
          const std::string& domainDir,
          const ActionSet& actions,
          unsigned int max_time = 0) {

		std::string ros_distro = std::getenv("ROS_DISTRO");

		if (ros_distro == "indigo") {
			return new Clingo4_2(incrementalVar, queryDir, domainDir, actions, max_time);
		}

		if (ros_distro == "kinetic") {
			return new Clingo4_5(incrementalVar, queryDir, domainDir.substr(0, domainDir.size()-1)+"_kinetic/", actions, max_time);
		}
	}

};

}


#endif