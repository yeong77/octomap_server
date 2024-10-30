#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <visualization_msgs/MarkerArray.h>

using namespace octomap;

// 전역 변수 count
int count = 0;

void updateLargeTreeWithSmallTree(OcTree& smallTree, OcTree& largeTree, OcTree& tree) {
    for (OcTree::iterator it = smallTree.begin(), end = smallTree.end(); it != end; ++it) {
        point3d coord = it.getCoordinate();

        OcTreeKey smallkey;
        OcTreeKey largekey;

        smallkey = smallTree.coordToKey(coord);
        OcTreeNode* smallNode = smallTree.search(smallkey);
        

        // 좌표를 키로 변환
        if (largeTree.coordToKeyChecked(coord, largekey)) {

            // ROS_INFO("Key for point (%f, %f, %f): [%u, %u, %u]", coord.x(), coord.y(), coord.z(), largekey.k[0], largekey.k[1], largekey.k[2]);
            OcTreeNode* largeNode = largeTree.search(largekey);
        
            if (largeNode != nullptr) {  // largeNode가 유효한지 확인 (largenode가 점유된 상태)
                printf("aaaa");
                tree.updateNode(coord, false);
                if(!smallTree.isNodeOccupied(smallNode)){ //작은게 점유되지 않음. 
                    printf("falseeeee");     
                }
            
            } else {
                tree.updateNode(coord, true);
                printf("Large node not found.\n");
            }

        } else {
            ROS_ERROR("Failed to convert coordinate to key.");
        }

    }
    
    smallTree.clear();
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "octomap_free_space_visualization");
    ros::NodeHandle nh;

    // 0.5m 해상도의 Octomap 트리 생성
    OcTree tree(0.5);
  
    if (argc < 3) {
        ROS_ERROR("Usage: %s <large_res_map.bt> <small_res_map.bt>", argv[0]);
        return -1;
    }

    // 큰 해상도 Octomap 파일 로드
    OcTree largeTree(1.0);  // 큰 해상도 트리 초기화
    if (!largeTree.readBinary(argv[1])) {
        ROS_ERROR("Failed to load large resolution Octomap from file: %s", argv[1]);
        return -1;
    }

    // 작은 해상도 Octomap 파일 로드
    OcTree smallTree(0.3);  // 작은 해상도 트리 초기화
    if (!smallTree.readBinary(argv[2])) {
        ROS_ERROR("Failed to load small resolution Octomap from file: %s", argv[2]);
        return -1;
    }
  

    updateLargeTreeWithSmallTree(smallTree, largeTree, tree);
    
        
   
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
        for (OcTree::iterator it = largeTree.begin(), end = largeTree.end(); it != end; ++it) {
            unsigned int idx = it.getDepth();
            // printf("Node depth: %u\n", it.getDepth());
            geometry_msgs::Point cube_center;
            
            if (largeTree.isNodeOccupied(*it)) {
                // 점유된 노드 시각화
                cube_center.x = it.getX();
                cube_center.y = it.getY();
                cube_center.z = it.getZ();
                // printf("x,y,z : %f, %f, %f", cube_center.x, cube_center.y, cube_center.z);
                marker_array.markers[idx].points.push_back(cube_center);
                marker_array.markers[idx].header.frame_id = "map";
                marker_array.markers[idx].header.stamp = ros::Time::now() ;
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
            }
        }

        for (OcTree::iterator it = tree.begin(), end = tree.end(); it != end; ++it) {
            // printf("Node depth: %u\n", it.getDepth());
            geometry_msgs::Point cube_center;
            unsigned int idx = it.getDepth();
            cube_center.x = it.getX();
            cube_center.y = it.getY();
            cube_center.z = it.getZ();
            if (tree.isNodeOccupied(*it)) {
                // 점유된 노드 시각화
                
                // printf("x,y,z : %f, %f, %f", cube_center.x, cube_center.y, cube_center.z);
                marker_array.markers[2].points.push_back(cube_center);
                marker_array.markers[2].header.frame_id = "map";
                marker_array.markers[2].header.stamp = ros::Time::now();
                marker_array.markers[2].ns = "free_large_space";
                marker_array.markers[2].type = visualization_msgs::Marker::CUBE_LIST;
                marker_array.markers[2].scale.x = it.getSize();
                marker_array.markers[2].scale.y = it.getSize();
                marker_array.markers[2].scale.z = it.getSize();
                marker_array.markers[2].color.r = 0.0;
                marker_array.markers[2].color.g = 0.0;
                marker_array.markers[2].color.b = 1.0;  // 자유 공간은 파란색으로 설정
                marker_array.markers[2].color.a = 1.0;  // 자유 공간은 반투명으로 설정
                marker_array.markers[2].action = visualization_msgs::Marker::ADD;
               
            } 
            // else {
            //     marker_array.markers[2].points.push_back(cube_center);
            //     marker_array.markers[2].header.frame_id = "map";
            //     marker_array.markers[2].header.stamp = ros::Time::now();
            //     marker_array.markers[2].ns = "free_small_space";
            //     marker_array.markers[2].type = visualization_msgs::Marker::CUBE_LIST;
            //     marker_array.markers[2].scale.x = it.getSize();
            //     marker_array.markers[2].scale.y = it.getSize();
            //     marker_array.markers[2].scale.z = it.getSize();
            //     marker_array.markers[2].color.r = 0.0;
            //     marker_array.markers[2].color.g = 0.0;
            //     marker_array.markers[2].color.b = 1.0;  // 자유 공간은 파란색으로 설정
            //     marker_array.markers[2].color.a = 0.0;  // 자유 공간은 반투명으로 설정
            //     marker_array.markers[2].action = visualization_msgs::Marker::ADD;
            // }

        }
        // 점유된 공간과 자유 공간 마커 발행
        marker_pub.publish(marker_array);
    }
    // std::string mapFilename = "/root/new_map.bt"; // 저장할 파일 경로 설정
    // if (tree.writeBinary(mapFilename)) {
    //     ROS_INFO("Octomap saved successfully to: %s", mapFilename.c_str());
    // } else {
    //     ROS_ERROR("Failed to save Octomap to: %s", mapFilename.c_str());
    // }
    // ros::spinOnce();
    // loop_rate.sleep();
    

   


    return 0;
}
