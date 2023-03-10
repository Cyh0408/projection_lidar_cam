#include <pcl/point_types.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>


#define YELLOW "\033[33m" // yellow
#define GREEN "\033[33m" // green
#define REND "\033[0m" << std::endl
#define WARN (std::cout << YELLOW)
#define INFO (std::cout << GREEN)

using namespace std;

struct EIGEN_ALIGN16 PointXYZRGBI
{
    PCL_ADD_POINT4D;
    PCL_ADD_RGB;
    float i;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGBI,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (uint8_t, r, r)
                                  (uint8_t, g, g)
                                  (uint8_t, b, b)
                                  (float, i, i))

class RsCamFusion
{
    private:
        ros::Subscriber camera_sub, lidar_sub;
        ros::Publisher fused_image_pub, colored_cloud_showpub;

        void lidar_callback(const sensor_msgs::PointCloud2ConstPtr &lidar_msg);
        void camera_callback(const sensor_msgs::ImageConstPtr &input_image_msg);

        void publishCloudtoShow(const ros::Publisher &cloudtoshow_pub, const std_msgs::Header &header,
                                     const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud);

        void publishImage(const ros::Publisher& image_pub, const std_msgs::Header& header, const cv::Mat image);

    private:
        cv::Mat intrinsic;
        cv::Mat extrinsic;
        cv::Mat distcoeff;
        cv::Size imageSize;
        
        Eigen::Matrix4d transform, inv_transform;
        cv::Mat rVec = cv::Mat::zeros(3, 1, CV_64FC1); // 旋转向量：3x1的零矩阵
        cv::Mat rMat = cv::Mat::eye(3, 1, CV_64FC1); // 3x1的单位矩阵
        cv::Mat tVec = cv::Mat::zeros(3, 1, CV_64FC1); // 平移向量：3x1的零矩阵
        bool show_colored_cloud;

        int color[21][3] = 
        {
            {255, 0, 0}, {255, 69, 0}, {255, 99, 71}, 
            {255, 140, 0}, {255, 165, 0}, {238, 173, 14},
            {255, 193, 37}, {255, 255, 0}, {255, 236, 139},
            {202, 255, 112}, {0, 255, 0}, {84, 255, 159},
            {127, 255, 212}, {0, 229, 238}, {152, 245, 255},
            {178, 223, 238}, {126, 192, 238}, {28, 134, 238},
            {0, 0, 255}, {72, 118, 255}, {122, 103, 238} 
        };

        float color_distance;
        int frame_count = 0;

    public:
        RsCamFusion(cv::Mat cam_intrinsic, cv::Mat lidar2cam_extrinsic, cv::Mat cam_distcoeff, cv::Size img_size, float color_dis, bool show_cloud)
        {
            intrinsic = cam_intrinsic;
            extrinsic = lidar2cam_extrinsic; // lidar2cam 外参
            distcoeff = cam_distcoeff; // cam内参畸变
            transform(0,0) = extrinsic.at<double>(0,0);
            transform(0,1) = extrinsic.at<double>(0,1);
            transform(0,2) = extrinsic.at<double>(0,2);
            transform(0,3) = extrinsic.at<double>(0,3);
            transform(1,0) = extrinsic.at<double>(1,0);
            transform(1,1) = extrinsic.at<double>(1,1);
            transform(1,2) = extrinsic.at<double>(1,2);
            transform(1,3) = extrinsic.at<double>(1,3);
            transform(2,0) = extrinsic.at<double>(2,0);
            transform(2,1) = extrinsic.at<double>(2,1);
            transform(2,2) = extrinsic.at<double>(2,2);
            transform(2,3) = extrinsic.at<double>(2,3);
            transform(3,0) = extrinsic.at<double>(3,0);
            transform(3,1) = extrinsic.at<double>(3,1);
            transform(3,2) = extrinsic.at<double>(3,2);
            transform(3,3) = extrinsic.at<double>(3,3);
            inv_transform = transform.inverse();
            imageSize = img_size;
            color_distance = color_dis;
            show_colored_cloud = show_cloud;
        }
    public:
        RsCamFusion(ros::NodeHandle &nh);
        ~RsCamFusion();
        void Spin();

        string camera_topic, lidar_topic;
        sensor_msgs::PointCloud2 input_cloud_msg;

    


};