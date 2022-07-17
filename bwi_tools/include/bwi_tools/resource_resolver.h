/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * 
 * Code and original copyright copied from the resource_retriever ROS package. 
 */

#ifndef BWI_TOOLS_RESOURCE_RESOLVER_H
#define BWI_TOOLS_RESOURCE_RESOLVER_H

#include <stdexcept>

#include <ros/package.h>

namespace bwi_tools {

  inline std::string resolveRosResource(const std::string &resource_string) {
    std::string mod_resource_string = resource_string;
    if (resource_string.find("package://") == 0)
    {
      mod_resource_string.erase(0, strlen("package://"));
      size_t pos = mod_resource_string.find("/");
      if (pos == std::string::npos) {
        throw std::runtime_error("Could not parse package:// format for resource: " + resource_string);
      }

      std::string package = mod_resource_string.substr(0, pos);
      mod_resource_string.erase(0, pos);
      std::string package_path = ros::package::getPath(package);

      if (package_path.empty()) {
        throw std::runtime_error("Package [" + package + "] does not exist for resource: " + resource_string);
      }

      mod_resource_string = package_path + mod_resource_string;
    }

    return mod_resource_string;
  }

} /* bwi_tools */


#endif /* end of include guard: BWI_TOOLS_RESOURCE_RESOLVER_H */
