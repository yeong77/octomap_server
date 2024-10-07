#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_server/OctomapServer.h>

using namespace octomap_server;
using namespace octomap;

// 큰 해상도의 Octree를 작은 해상도의 Octree에 병합하는 함수
// 작은 해상도를 기준으로 하고, 큰 해상도의 노드를 여러 작은 노드로 나누어 병합
void mergeLargeIntoSmall(OcTree& smallTree, OcTree& largeTree) {
    double smallRes = smallTree.getResolution();  // 작은 해상도의 해상도
    double largeRes = largeTree.getResolution();  // 큰 해상도의 해상도

    // 큰 트리의 모든 리프 노드를 순회하며 작은 트리에 추가
    for (OcTree::leaf_iterator it = largeTree.begin_leafs(), end = largeTree.end_leafs(); it != end; ++it) {
        if (largeTree.isNodeOccupied(*it)) {  // 큰 트리에서 점유된 노드만 병합
            point3d coord = it.getCoordinate();  // 큰 트리 노드의 좌표

            // 큰 해상도의 노드를 작은 해상도에 맞춰 분할하여 병합
            if (largeRes > smallRes) {
                // 해상도 비율 계산 (큰 노드를 작은 노드로 나눌 비율)
                int ratio = largeRes / smallRes;

                // 큰 해상도 노드를 작은 해상도에 맞춰 여러 노드로 나누어 삽입
                for (int i = 0; i < ratio; i++) {
                    for (int j = 0; j < ratio; j++) {
                        for (int k = 0; k < ratio; k++) {
                            // 작은 해상도의 좌표 계산
                            point3d subCoord = coord + point3d(i * smallRes, j * smallRes, k * smallRes);
                            smallTree.updateNode(subCoord, true);  // 작은 해상도의 노드를 병합
                        }
                    }
                }
            } else {
                // 해상도가 같거나 작은 경우 작은 트리에 바로 병합
                smallTree.updateNode(coord, true);
            }
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "octomap_merge");
    ros::NodeHandle nh;

    if (argc < 3) {
        ROS_ERROR("Usage: %s <small_res_map.bt> <large_res_map.bt>", argv[0]);
        return -1;
    }

    // 첫 번째 인자로 받은 작은 해상도 Octomap 파일 로드
    OcTree smallTree(0.05);  // 임시로 초기 해상도 설정 (작은 해상도)
    if (!smallTree.readBinary(argv[1])) {  // argv[1]은 작은 해상도 맵 파일 경로
        ROS_ERROR("Failed to load small resolution Octomap from file: %s", argv[1]);
        return -1;
    }

    // 두 번째 인자로 받은 큰 해상도 Octomap 파일 로드
    OcTree largeTree(0.1);  // 임시로 초기 해상도 설정 (큰 해상도)
    if (!largeTree.readBinary(argv[2])) {  // argv[2]은 큰 해상도 맵 파일 경로
        ROS_ERROR("Failed to load large resolution Octomap from file: %s", argv[2]);
        return -1;
    }

    // 큰 해상도의 트리를 작은 해상도의 트리에 병합
    mergeLargeIntoSmall(smallTree, largeTree);

    // 병합된 트리를 ROS 주제로 게시하는 OctomapServer 생성
    OctomapServer server(nh);

    // 병합된 Octomap을 OctomapServer로 로드하고 게시
    std::string mergedFilename = "/root/merged_map.bt";
    smallTree.writeBinary(mergedFilename);  // 병합된 Octomap을 파일로 저장
    
    if (!server.openFile(mergedFilename)) {  // OctomapServer가 병합된 파일을 로드
        ROS_ERROR("Failed to open merged Octomap file: %s", mergedFilename.c_str());
        return -1;
    }

    // ROS 루프를 통해 지속적으로 주제 게시
    ros::spin();

    return 0;
}
