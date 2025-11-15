/**
 * board_node.cpp
 * 专为 ROS1 Noetic 编写的 ArUco Board 检测节点
 * (已修改：将 drawAxis 移至 publishPose 以匹配中心点)
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class ArucoBoardNode {
public:
    ArucoBoardNode() : it_(nh_) {
        // 1. 获取参数 (从 launch 文件加载)
        ros::NodeHandle private_nh("~");
        
        private_nh.param<int>("markers_x", markers_x_, 2);
        private_nh.param<int>("markers_y", markers_y_, 2);
        private_nh.param<double>("marker_length", marker_length_, 0.16);
        private_nh.param<double>("marker_separation", marker_separation_, 0.04);
        private_nh.param<int>("dictionary_id", dictionary_id_, 10); // 默认为 DICT_4X4_50 (10)
        private_nh.param<bool>("show_debug_image", show_debug_image_, false);
        private_nh.param<std::string>("camera_topic", camera_topic_, "/camera/color/image_raw");
        private_nh.param<std::string>("camera_info_topic", camera_info_topic_, "/camera/color/camera_info");
        private_nh.param<std::string>("pose_topic", pose_topic_, "board_detect/pose");
        private_nh.param<std::string>("debug_topic", debug_topic_, "board_detect/debug_image");

        // 2. 初始化 ArUco 字典和板子
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionary_id_));
        // OpenCV 4.x 写法: create
        board_ = cv::aruco::GridBoard::create(markers_x_, markers_y_, (float)marker_length_, (float)marker_separation_, dictionary_);
        parameters_ = cv::aruco::DetectorParameters::create();

        // 3. 订阅与发布 
        image_sub_ = it_.subscribe(camera_topic_, 1, &ArucoBoardNode::imageCallback, this);
        info_sub_ = nh_.subscribe(camera_info_topic_, 1, &ArucoBoardNode::camInfoCallback, this);
        
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic_, 10);
        debug_pub_ = it_.advertise(debug_topic_, 1);

        ROS_INFO("ArUco Board Node Started. Waiting for images...");
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher debug_pub_;
    ros::Subscriber info_sub_;
    ros::Publisher pose_pub_;

    // 图像话题名称参数
    std::string camera_topic_, camera_info_topic_;
    // 发布话题名称参数
    std::string pose_topic_ , debug_topic_;
    // ArUco 参数
    int markers_x_, markers_y_;
    double marker_length_, marker_separation_;
    int dictionary_id_;
    bool show_debug_image_;

    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::GridBoard> board_;
    cv::Ptr<cv::aruco::DetectorParameters> parameters_;

    // 相机内参
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    bool has_camera_info_ = false;

    void camInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
        if (has_camera_info_) return; // 只需要获取一次

        camera_matrix_ = cv::Mat(3, 3, CV_64F);
        for (int i = 0; i < 9; i++) camera_matrix_.at<double>(i / 3, i % 3) = msg->K[i];

        dist_coeffs_ = cv::Mat(msg->D.size(), 1, CV_64F);
        for (int i = 0; i < msg->D.size(); i++) dist_coeffs_.at<double>(i) = msg->D[i];

        has_camera_info_ = true;
        ROS_INFO("Camera Intrinsics Received.");
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        if (!has_camera_info_) {
            ROS_WARN_THROTTLE(2, "Waiting for camera info...");
            return;
        }

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;

        // 检测标记
        cv::aruco::detectMarkers(cv_ptr->image, dictionary_, corners, ids, parameters_);

        if (ids.size() > 0) {
            // 估计板子位姿
            cv::Vec3d rvec, tvec;
            // estimatePoseBoard 返回使用的标记数量
            int valid = cv::aruco::estimatePoseBoard(corners, ids, board_, camera_matrix_, dist_coeffs_, rvec, tvec);

            if (valid > 0) {
                publishPose(rvec, tvec, msg->header, cv_ptr);
            }
            
            // (画标记框的代码保留)
            if (debug_pub_.getNumSubscribers() > 0 || show_debug_image_) {
                cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);
            }
        }

        // 发布调试图像
        if (debug_pub_.getNumSubscribers() > 0) {
            debug_pub_.publish(cv_ptr->toImageMsg());
        }
        
    }

    // 接收 cv_ptr
    void publishPose(cv::Vec3d rvec, cv::Vec3d tvec, std_msgs::Header header, cv_bridge::CvImagePtr cv_ptr) {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header = header; // 使用图像的时间戳和Frame ID

        // (原始的 tvec 赋值，是角落坐标，会被后面覆盖)
        // pose_msg.pose.position.x = tvec[0];
        // pose_msg.pose.position.y = tvec[1];
        // pose_msg.pose.position.z = tvec[2];

        // 旋转向量转四元数
        tf2::Quaternion q;
        double angle = cv::norm(rvec);
        if (angle > 0) {
            cv::Vec3d axis = rvec / angle;
            q.setRotation(tf2::Vector3(axis[0], axis[1], axis[2]), angle);
        } else {
            q.setRPY(0, 0, 0);
        }
        
        // 1. 计算板子物理总宽和总高
        double total_width = markers_x_ * marker_length_ + (markers_x_ - 1) * marker_separation_;
        double total_height = markers_y_ * marker_length_ + (markers_y_ - 1) * marker_separation_;

        // 2. 定义中心点在当前"角落原点"坐标系下的位置
        // 假设原点在左下角，X向右，Y向上：
        cv::Mat offset_vector = (cv::Mat_<double>(3, 1) << total_width / 2.0, total_height / 2.0, 0);

        // 3. 将这个偏移量应用旋转 (从板子系转到相机系)
        cv::Mat rot_matrix;
        cv::Rodrigues(rvec, rot_matrix);
        
        // tvec_center = tvec_corner + R * offset
        cv::Mat tvec_center_mat = (cv::Mat_<double>(3, 1) << tvec[0], tvec[1], tvec[2]) + rot_matrix * offset_vector;

        // 4. 赋值给消息 (使用中心点坐标)
        pose_msg.pose.position.x = tvec_center_mat.at<double>(0);
        pose_msg.pose.position.y = tvec_center_mat.at<double>(1);
        pose_msg.pose.position.z = tvec_center_mat.at<double>(2);

        // 在这里使用中心点坐标 (tvec_center_mat) 来绘制坐标轴
        if (cv_ptr && (debug_pub_.getNumSubscribers() > 0 || show_debug_image_)) {
            // 将 Mat 转回 Vec3d 以便 drawAxis 使用
            cv::Vec3d tvec_center(tvec_center_mat.at<double>(0), tvec_center_mat.at<double>(1), tvec_center_mat.at<double>(2));
            cv::aruco::drawAxis(cv_ptr->image, camera_matrix_, dist_coeffs_, rvec, tvec_center, 0.1); // 轴长0.1m
        }

        // 赋值四元数
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();

        pose_pub_.publish(pose_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "aruco_board_detect_node");
    ArucoBoardNode node;
    ros::spin();
    return 0;
}