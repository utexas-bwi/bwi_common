#pragma once

#include <actasp/reasoners/Clingo4_2.h>
#include <actasp/reasoners/Clingo4_5.h>
#include <actasp/reasoners/Clingo5_2.h>
#include <boost/filesystem.hpp>
#include <actasp/filesystem_utils.h>

namespace actasp {

struct Clingo {

	static FilteringQueryGenerator* getQueryGenerator(const std::string& incrementalVar,
          const std::string& linkDir,
					const std::vector<std::string>& copyFiles,
          const ActionSet& actions,
          unsigned int max_time = 0) {

		std::string ros_distro = std::getenv("ROS_DISTRO");

    // TODO: This should be tied to which version of clingo is available, not which distro we're on
		if (ros_distro == "indigo") {
			auto indigoPath = linkDir.substr(0, linkDir.size()-1)+"_indigo/";
			return new Clingo4_2(incrementalVar, dirToAllAspFilesInDir(indigoPath), copyFiles, actions, max_time);
		}

		if (ros_distro == "kinetic") {
			return new Clingo4_5(incrementalVar, dirToAllAspFilesInDir(linkDir), copyFiles, actions, max_time);
		}

		if (ros_distro == "melodic") {
			return new Clingo5_2(incrementalVar, dirToAllAspFilesInDir(linkDir), copyFiles, actions, max_time);
		}
		assert(false);
	}

};

}
