#include "cam_lidar.h"

RsCamFusion::RsCamFusion(ros::NodeHandle &nh)
{
    camera_sub = nh.subscribe("/zed/zed_node/left/image_rect_color", 30, &RsCamFusion::camera_callback, this); 
    lidar_sub = nh.subscribe("/rslidar_points", 10, &RsCamFusion::lidar_callback, this); 

    fused_image_pub = nh.advertise<sensor_msgs::Image>("/fused_image", 10);
    colored_cloud_showpub = nh.advertise<sensor_msgs::PointCloud2>("/colored_cloud_toshow", 10);

    ros::spin();
}

RsCamFusion::~RsCamFusion(){}

void RsCamFusion::Spin(){}

void RsCamFusion::publishCloudtoShow(const ros::Publisher &cloudtoshow_pub, const std_msgs::Header &header,
                                     const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
{
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud, output_msg);
    output_msg.header = header;
    cloudtoshow_pub.publish(output_msg);
}

void RsCamFusion::publishImage(const ros::Publisher& image_pub, const std_msgs::Header& header, const cv::Mat image)
{
    cv_bridge::CvImage output_image;
    output_image.header.frame_id = header.frame_id;
    output_image.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
    output_image.image = image;
    image_pub.publish(output_image);
}

void RsCamFusion::lidar_callback(const sensor_msgs::PointCloud2ConstPtr &lidar_msg)
{
    input_cloud_msg = *lidar_msg;
}

void RsCamFusion::camera_callback(const sensor_msgs::ImageConstPtr &input_image_msg)
{
    cv::Mat input_image;
    cv::Mat undistorted_image;
    cv_bridge::CvImagePtr cv_ptr;

    std_msgs::Header image_header = input_image_msg->header;
    std_msgs::Header cloud_header = input_cloud_msg.header;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(input_image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception e)
    {
        ROS_ERROR_STREAM("Cv_bridge Exception:" << e.what());
        return;
    }
    input_image = cv_ptr->image;

    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(input_cloud_msg, *input_cloud_ptr);
    // if (input_cloud_ptr->size() == 0) 
    // {
    //     WARN << "input cloud is empty, please check it out!" << REND;
    // }
    // cout << "size: " << input_cloud_ptr->size() << endl;

    // // 将点云从雷达坐标系转换到相机坐标系
    // // 雷达：x前，y左，z上； 相机：z前，x右，z下
    // pcl::transformPointCloud (*input_cloud_ptr, *transformed_cloud, transform);

    // std::vector<cv::Point3d> lidar_points;
    // std::vector<cv::Scalar> dis_color;
    // std::vector<float> intensity;
    // std::vector<cv::Point2d> imagePoints;

    // for(int i=0; i<transformed_cloud->points.size(); i++)
    // {
    //     if(transformed_cloud->points[i].z > 0)
    //     {
    //         lidar_points.push_back(cv::Point3d(transformed_cloud->points[i].x, transformed_cloud->points[i].y, transformed_cloud->points[i].z));
    //         int color_order = int(transformed_cloud->points[i].z / color_distance);
    //         if(color_order > 20)
    //         {
    //             color_order = 20;
    //         }
    //         dis_color.push_back(cv::Scalar(color[color_order][2], color[color_order][1], color[color_order][0]));
    //         intensity.push_back(transformed_cloud->points[i].intensity);
    //     }
    // }

    // // 将点云从相机坐标系投影到图片坐标系(向右：X+， 向下：Y+)
    // cv::projectPoints(lidar_points, rMat, tVec, intrinsic, distcoeff, imagePoints);
    // pcl::PointCloud<PointXYZRGBI>::Ptr colored_cloud(new pcl::PointCloud<PointXYZRGBI>);
    // pcl::PointCloud<PointXYZRGBI>::Ptr colored_cloud_transback(new pcl::PointCloud<PointXYZRGBI>);
    // cv::Mat image_to_show = input_image.clone();

    // for(int i=0; i<imagePoints.size(); i++)
    // {
    //     if(imagePoints[i].x >= 0 && imagePoints[i].x < 1920 && imagePoints[i].y >= 0 && imagePoints[i].y < 1200)
    //     {
    //         cv::circle(image_to_show, imagePoints[i], 1, dis_color[i], 2, 8, 0);
    //         PointXYZRGBI point;
    //         point.x = lidar_points[i].x;
    //         point.y = lidar_points[i].y;                                                        //to create colored point clouds
    //         point.z = lidar_points[i].z;
    //         point.r = input_image.at<cv::Vec3b>(imagePoints[i].y, imagePoints[i].x)[2];
    //         point.g = input_image.at<cv::Vec3b>(imagePoints[i].y, imagePoints[i].x)[1];
    //         point.b = input_image.at<cv::Vec3b>(imagePoints[i].y, imagePoints[i].x)[0];
    //         point.i = intensity[i];
    //         colored_cloud->points.push_back(point);
    //     }
    // }

    // // 将带颜色的点云从相机坐标系转换到雷达坐标系
    // pcl::transformPointCloud(*colored_cloud, *colored_cloud_transback, inv_transform);

    // if(show_colored_cloud)
    // {
    //     pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud_toshow (new pcl::PointCloud<pcl::PointXYZRGB>);
    //     for(int i=0; i<colored_cloud_transback->points.size();i++)
    //     {
    //         pcl::PointXYZRGB point;
    //         point.x = colored_cloud_transback->points[i].x;                                                        
    //         point.y = colored_cloud_transback->points[i].y;                                                        
    //         point.z = colored_cloud_transback->points[i].z;
    //         point.r = colored_cloud_transback->points[i].r;
    //         point.g = colored_cloud_transback->points[i].g;
    //         point.b = colored_cloud_transback->points[i].b;
    //         colored_cloud_toshow->points.push_back (point);
    //     }
    // publishCloudtoShow(colored_cloud_showpub, cloud_header, colored_cloud_toshow);
    // }

    // publishImage(fused_image_pub, image_header, image_to_show);

    // frame_count = frame_count + 1;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cam_lidar");
    ros::NodeHandle nh; // 全局命名空间
    ros::NodeHandle priv_nh("~"); //局部命名空间

    RsCamFusion core(nh);

    // string camera_topic, lidar_topic;
    // string config_path, file_name;
    
    // float color_dis;
    // bool show_cloud;
    // if (priv_nh.hasParam("calib_file_path") && priv_nh.hasParam("file_name"))
    // {
    //     priv_nh.getParam("camera_topic", camera_topic);
    //     priv_nh.getParam("lidar_topic", lidar_topic);
    //     priv_nh.getParam("calib_file_path", config_path);
    //     priv_nh.getParam("file_name", file_name);
    //     priv_nh.getParam("color_distance", color_dis);
    //     priv_nh.getParam("show_colored_cloud", show_cloud);
    // }
    // else 
    // {
    //     WARN << "Config file is empty!" << REND;
    //     return 0;
    // }

    // INFO << "config path: " << config_path << REND;
    // INFO << "config file: " << file_name << REND;

    // string config_file_name = config_path + "/" + file_name;
    // cv::FileStorage fs_reader(config_file_name, cv::FileStorage::READ);

    // cv::Mat cam_intrinsic, lidar2cam_extrinsic, cam_distcoeff;
    // cv::Size img_size;
    // fs_reader["CameraMat"] >> cam_intrinsic;
    // fs_reader["CameraExtrinsicMat"] >> lidar2cam_extrinsic;
    // fs_reader["DistCoeff"] >> cam_distcoeff;
    // fs_reader["ImageSize"] >> img_size;
    // fs_reader.release();

    // if (lidar_topic.empty() || camera_topic.empty())
    // {
    //     WARN << "Sensor topic is empty!" << REND;
    //     return 0;
    // }

    // INFO << "lidar topic: " << lidar_topic << REND;
    // INFO << "camera topic: " << camera_topic << REND;
    // INFO << "camera intrinsic matrix: " << cam_intrinsic << REND;
    // INFO << "lidar2cam entrinsic matrix: " << lidar2cam_extrinsic << REND;

    return 0;

}