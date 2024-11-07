#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

class OrbslamPathPublisher {
public:
    OrbslamPathPublisher() {
        // Path 퍼블리셔 초기화
        path_pub_ = nh_.advertise<nav_msgs::Path>("/orb_slam2_rgbd/path", 10);
        
        // ORB-SLAM2 위치 토픽 구독 (PoseStamped 타입으로 변경)
        pose_sub_ = nh_.subscribe("/orb_slam2_rgbd/pose", 10, &OrbslamPathPublisher::poseCallback, this);
        
        // Path 메시지 초기화
        path_msg_.header.frame_id = "map";  // 혹은 "world"로 변경 가능
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // Path 메시지에 새로운 위치 추가
        path_msg_.poses.push_back(*msg);
        path_msg_.header.stamp = ros::Time::now();

        // Path 퍼블리시
        path_pub_.publish(path_msg_);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher path_pub_;
    ros::Subscriber pose_sub_;
    nav_msgs::Path path_msg_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "orbslam_path_publisher");

    OrbslamPathPublisher orbslam_path_publisher;

    ros::spin();
    return 0;
}
