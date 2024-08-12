/*
 * Copyright (c) 2010-2013, A. Hornung, University of Freiburg
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
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <fstream>

#include <octomap_msgs/GetOctomap.h>
using octomap_msgs::GetOctomap;

#define USAGE "\nUSAGE: octomap_saver [-f] <mapfile.[bt|ot]>\n" \
                "  -f: Query for the full occupancy octree, instead of just the compact binary one\n" \
		"  mapfile.bt: filename of map to be saved (.bt: binary tree, .ot: general octree)\n"

using namespace std;
using namespace octomap;

class MapSaver{
public:
  MapSaver(const std::string& mapname1, const std::string& mapname2, bool full){
    ros::NodeHandle n;
    std::string servname1 = "octomap_binary";
    if (full)
      servname1 = "octomap_full";
    ROS_INFO("Requesting the map from %s...", n.resolveName(servname1).c_str());
    GetOctomap::Request req1;
    GetOctomap::Response resp1;
    while(n.ok() && !ros::service::call(servname1, req1, resp1))
    {
      ROS_WARN("Request to %s failed; trying again...", n.resolveName(servname1).c_str());
      usleep(1000000);
    }

    if (n.ok()){ // skip when CTRL-C
      AbstractOcTree* tree1 = octomap_msgs::msgToMap(resp1.map);
      AbstractOccupancyOcTree* octree1 = NULL;
      if (tree1){
        octree1 = dynamic_cast<AbstractOccupancyOcTree*>(tree1);
      } else {
        ROS_ERROR("Error creating octree from received message");
        if (resp1.map.id == "ColorOcTree")
          ROS_WARN("You requested a binary map for a ColorOcTree - this is currently not supported. Please add -f to request a full map");
      }

      if (octree1){
        //ROS_INFO("Map received (%zu nodes, %f m res), saving to %s", octree->size(), octree->getResolution(), mapname.c_str());
        
        std::string suffix1 = mapname1.substr(mapname1.length()-3, 3);
        if (suffix1 == ".bt"){ // write to binary file:
          if (!octree1->writeBinary(mapname1)){
            ROS_ERROR("Error writing to file %s", mapname1.c_str());
          }
        } else if (suffix1 == ".ot"){ // write to full .ot file:
          if (!octree1->write(mapname1)){
            ROS_ERROR("Error writing to file %s", mapname1.c_str());
          }
        } else{
          ROS_ERROR("Unknown file extension, must be either .bt or .ot");
        }
      } else{
        ROS_ERROR("Error reading OcTree from stream");
      }

      delete octree1;

    }

    std::string servname2 = "octomap_binary1";
    if (full)
      servname2 = "octomap_full1";
    ROS_INFO("Requesting the map from %s...", n.resolveName(servname2).c_str());
    GetOctomap::Request req2;
    GetOctomap::Response resp2;
    while(n.ok() && !ros::service::call(servname2, req2, resp2))
    {
      ROS_WARN("Request to %s failed; trying again...", n.resolveName(servname2).c_str());
      usleep(1000000);
    }

    if (n.ok()){ // skip when CTRL-C
      AbstractOcTree* tree2 = octomap_msgs::msgToMap(resp2.map);
      AbstractOccupancyOcTree* octree2 = NULL;
      if (tree2){
        octree2 = dynamic_cast<AbstractOccupancyOcTree*>(tree2);
      } else {
        ROS_ERROR("Error creating octree2 from received message");
        if (resp2.map.id == "ColorOcTree")
          ROS_WARN("You requested a binary map for a ColorOcTree - this is currently not supported. Please add -f to request a full map");
      }

      if (octree2){
        std::string suffix2 = mapname2.substr(mapname2.length()-3, 3);
        if (suffix2 == ".bt"){ // write to binary file:
          if (!octree2->writeBinary(mapname2)){
            ROS_ERROR("Error writing to file %s", mapname2.c_str());
          }
        } else if (suffix2 == ".ot"){ // write to full .ot file:
          if (!octree2->write(mapname2)){
            ROS_ERROR("Error writing to file %s", mapname2.c_str());
          }
        } else{
          ROS_ERROR("Unknown file extension, must be either .bt or .ot");
        }
      } else{
        ROS_ERROR("Error reading OcTree from stream");
      }

      delete octree2;
    }
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "octomap_saver");
  std::string mapFilename1("");
  std::string mapFilename2("");
  bool fullmap = false;
  if (argc == 4 && strcmp(argv[1], "-f")==0){
    fullmap = true;
    mapFilename1 = std::string(argv[2]);
    mapFilename2 = std::string(argv[3]);
    
  } else if (argc == 3){
    mapFilename1 = std::string(argv[1]);
    mapFilename2 = std::string(argv[2]);
  } else{
    ROS_ERROR("%s", USAGE);
    exit(1);
  }

  try{
    MapSaver ms(mapFilename1, mapFilename2 , fullmap);
  }catch(std::runtime_error& e){
    ROS_ERROR("octomap_saver exception: %s", e.what());
    exit(2);
  }

  exit(0);
}


