/**
* octomap_server: A Tool to serve 3D OctoMaps in ROS (binary and as visualization)
* (inspired by the ROS map_saver)
* @author A. Hornung, University of Freiburg, Copyright (C) 2009 - 2012.
* @see http://octomap.sourceforge.net/
* License: BSD
*/

/*
 * Copyright (c) 2009-2012, A. Hornung, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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
 */


#include <ros/ros.h>
#include <octomap_server/OctomapServer.h>

#define USAGE "\nUSAGE: octomap_server <map.[bt|ot]>\n" \
        "  map.bt: inital octomap 3D map file to read\n"

using namespace octomap_server;

int main(int argc, char** argv){
  ros::init(argc, argv, "octomap_server");
  const ros::NodeHandle& private_nh = ros::NodeHandle("~");
  std::string mapFilename1(""), mapFilename2(""), mapFilenameParam1(""), mapFilenameParam2("");

  if (argc > 3 || (argc == 2 && std::string(argv[1]) == "-h")){
    ROS_ERROR("%s", USAGE);
    exit(-1);
  }

  OctomapServer server;
  ros::spinOnce();

  // if (argc == 2){
  //   mapFilename = std::string(argv[1]);
  // }

  if (argc == 2 || argc == 3){
    mapFilename1 = std::string(argv[1]);
    if (argc == 3) {
      mapFilename2 = std::string(argv[2]);
    }
  }

  // if (private_nh.getParam("map_file", mapFilenameParam)) {
  //   if (mapFilename != "") {
  //     ROS_WARN("map_file is specified by the argument '%s' and rosparam '%s'. now loads '%s'", mapFilename.c_str(), mapFilenameParam.c_str(), mapFilename.c_str());
  //   } else {
  //     mapFilename = mapFilenameParam;
  //   }
  // }

  if (private_nh.getParam("map_file_1", mapFilenameParam1)) {
    if (mapFilename1 != "") {
      ROS_WARN("map_file_1 is specified by the argument '%s' and rosparam '%s'. Now loading '%s'", mapFilename1.c_str(), mapFilenameParam1.c_str(), mapFilename1.c_str());
    } else {
      mapFilename1 = mapFilenameParam1;
    }
  }

  if (private_nh.getParam("map_file_2", mapFilenameParam2)) {
    if (mapFilename2 != "") {
      ROS_WARN("map_file_2 is specified by the argument '%s' and rosparam '%s'. Now loading '%s'", mapFilename2.c_str(), mapFilenameParam2.c_str(), mapFilename2.c_str());
    } else {
      mapFilename2 = mapFilenameParam2;
    }
  }

  // if (mapFilename != "") {
  //   if (!server.openFile(mapFilename)){
  //     ROS_ERROR("Could not open file %s", mapFilename.c_str());
  //     exit(1);
  //   }
  // }

  // Load the first map
  if (mapFilename1 != "") {
    if (!server.openFile(mapFilename1)){
      ROS_ERROR("Could not open file %s", mapFilename1.c_str());
      exit(1);
    }
  }

  // Load the second map if provided
  if (mapFilename2 != "") {
    if (!server.openFile(mapFilename2)){
      ROS_ERROR("Could not open file %s", mapFilename2.c_str());
      exit(1);
    }
  }

  try{
    ros::spin();
  }catch(std::runtime_error& e){
    ROS_ERROR("octomap_server exception: %s", e.what());
    return -1;
  }

  return 0;
}
