#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <visualization_msgs/MarkerArray.h>

using namespace octomap;

// 전역 변수 count
int count = 0;

int main(int argc, char** argv) {
    ros::init(argc, argv, "octomap_free_space_visualization");
    ros::NodeHandle nh;

    // 1.0m 해상도의 Octomap 트리 생성
    OcTree tree(1.0);

    // 좌표 (idx.0, 3.0, 1.0)과 (idx.0, 3.0, idx.0) 생성
    // point3d coord(4.5, 4.5, 0.5);

    point3d coord2(4.5, 4.5, 1.5);
    // point3d childCoord(4.25, 4.25, 0.25);
    point3d coord3(4.5, 4.5, 0.5);
    
    
    
    // OcTreeNode* node = tree.search(coord3);
    // if (!tree.nodeHasChildren(node)) {
    //     tree.expandNode(node);  // 자식 노드로 확장
    //     point3d childCoord(4.25, 4.25, 0.25);
    //     // point3d childCoord2(4.75, 4.25, 0.25);
    //     tree.updateNode(childCoord, true);
    //     // tree.updateNode(childCoord2, true);

    // }    

    // if(tree.isNodeOccupied(node)){
    //     printf("yessss");
    // }
    // else{
    //     printf("noooooo");
    // }
    
    // tree.updateNode(coord3, true); // 좌표 (idx.0, 3.0, idx.0) 점유
    tree.updateNode(coord3, true);

    tree.updateNode(coord3, false);
    tree.clear();

    OcTreeNode* parentNode = tree.search(coord3);
    if (parentNode) {
        if (tree.isNodeOccupied(parentNode)) {
            ROS_INFO("Parent node is occupied.");
        } else {
            ROS_INFO("Parent node is free.");
        }
    }
    else{
        printf("clear");
    }
    
        
   
    // ROS 퍼블리셔 설정 (Octomap 전체, 자유 공간, 마커 발행)
    ros::Publisher octomap_pub = nh.advertise<octomap_msgs::Octomap>("octomap_full", 1);
    ros::Publisher free_space_pub = nh.advertise<visualization_msgs::MarkerArray>("free_space_markers", 1);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("octomap_markers", 1);

    // 실시간으로 발행할 주기 설정
    ros::Rate loop_rate(1.0);  // 1Hz로 발행

    while (ros::ok()) {
        // Octomap 메시지로 변환하여 발행
        octomap_msgs::Octomap octomap_msg;
        octomap_msgs::fullMapToMsg(tree, octomap_msg);
        octomap_msg.header.frame_id = "map";
        octomap_msg.header.stamp = ros::Time::now();
        octomap_pub.publish(octomap_msg);

        // 실시간 마커 생성 및 발행
        visualization_msgs::MarkerArray marker_array;
        marker_array.markers.resize(tree.getTreeDepth() + 1);  // 트리 깊이만큼 마커 생성

       
        // 점유된 노드와 자유 공간 노드에 대한 마커 추가
        for (OcTree::iterator it = tree.begin(), end = tree.end(); it != end; ++it) {
            unsigned int idx = it.getDepth();
            // printf("Node depth: %u\n", it.getDepth());
            geometry_msgs::Point cube_center;
            cube_center.x = it.getX();
            cube_center.y = it.getY();
            cube_center.z = it.getZ();
            
            if (tree.isNodeOccupied(*it)) {
                // 점유된 노드 시각화
                
                // printf("x,y,z : %f, %f, %f", cube_center.x, cube_center.y, cube_center.z);
                marker_array.markers[idx].points.push_back(cube_center);
                marker_array.markers[idx].header.frame_id = "map";
                marker_array.markers[idx].header.stamp = ros::Time::now();
                marker_array.markers[idx].ns = "occupied_space";
                marker_array.markers[idx].type = visualization_msgs::Marker::CUBE_LIST;
                marker_array.markers[idx].scale.x = it.getSize();
                marker_array.markers[idx].scale.y = it.getSize();
                marker_array.markers[idx].scale.z = it.getSize();
                marker_array.markers[idx].color.r = 1.0;
                marker_array.markers[idx].color.g = 0.0;
                marker_array.markers[idx].color.b = 0.0;
                marker_array.markers[idx].color.a = 1.0;
                marker_array.markers[idx].action = visualization_msgs::Marker::ADD;
                count+=1;
                // printf("count %d", count);
            } else {
                // 자유 공간 노드 시각화
                marker_array.markers[idx].points.push_back(cube_center);
                marker_array.markers[idx].header.frame_id = "map";
                marker_array.markers[idx].header.stamp = ros::Time::now();
                marker_array.markers[idx].ns = "free_space";
                marker_array.markers[idx].type = visualization_msgs::Marker::CUBE_LIST;
                marker_array.markers[idx].scale.x = 1.0;
                marker_array.markers[idx].scale.y = 1.0;
                marker_array.markers[idx].scale.z = 1.0;
                marker_array.markers[idx].color.r = 0.0;
                marker_array.markers[idx].color.g = 0.0;
                marker_array.markers[idx].color.b = 1.0;  // 자유 공간은 파란색으로 설정
                marker_array.markers[idx].color.a = 0.5;  // 자유 공간은 반투명으로 설정
                marker_array.markers[idx].action = visualization_msgs::Marker::ADD;
            }
            
        }
    
        // 점유된 공간과 자유 공간 마커 발행
        marker_pub.publish(marker_array);
        // free_space_pub.publish(marker_array);

    std::string mapFilename = "/root/new_map.bt"; // 저장할 파일 경로 설정
    if (tree.writeBinary(mapFilename)) {
        ROS_INFO("Octomap saved successfully to: %s", mapFilename.c_str());
    } else {
        ROS_ERROR("Failed to save Octomap to: %s", mapFilename.c_str());
    }
    ros::spinOnce();
    loop_rate.sleep();
    }



    return 0;
}