#include "projection.h"

class projection
{
    private:
        
        cv::Mat image_proj;
        image_transport::Publisher image_publisher;
        tf::TransformBroadcaster tr_br;

        float maxX = 25.0, maxY = 6.0, minZ = -1.4;

    public:
        projection()
        {
            ros::NodeHandle nh("~");
            initParams();
            message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, i_params.camera_topic, 30);
            message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub(nh, i_params.lidar_topic, 10);

            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
            message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(5), image_sub, lidar_sub);
            sync.registerCallback(boost::bind(&projection::projection_callback, this, _1, _2));

            image_transport::ImageTransport imageTranspors(nh);
            image_publisher = imageTranspors.advertise("/project_pc_image", 20);
            ros::spin();
        }

        void initParams()
        {
            std::string pkg_loc = ros::package::getPath("projection");
            std::ifstream infile(pkg_loc + "/cfg/initial_params.txt");
            infile >> i_params.camera_topic;
            infile >> i_params.lidar_topic;

            double_t camtocam[12];
            double_t cameraIn[16];
            double_t RT[16];

            for (int i = 0; i < 16; i++)
            {
                infile >> camtocam[i];
            }
            cv::Mat(4, 4, 6, &camtocam).copyTo(i_params.camtocam_mat);

            for (int i = 0; i < 12; i++)
            {
                infile >> cameraIn[i];
            }
            cv::Mat(4, 4, 6, &cameraIn).copyTo(i_params.cameraIn);

            for (int i = 0; i < 16; i++)
            {
                infile >> RT[i];
            }
            cv::Mat(4, 4, 6, &RT).copyTo(i_params.RT);
            std::cout << i_params.RT << std::endl;
        }

        void projection_callback(const sensor_msgs::ImageConstPtr &img_msg, const sensor_msgs::PointCloud2ConstPtr &lidar_msg)
        {
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(img_msg, "bgr8");
            }
            catch(cv_bridge::Exception &e)
            {
                return;
            }
            cv::Mat raw_img = cv_ptr->image;

            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromROSMsg(*lidar_msg, *cloud);

            cv::Mat visImg = raw_img.clone();
            cv::Mat overlay = visImg.clone();
            // std::cout << "Get lidar and image data" << std::endl;

            cv::Mat X(4,1, cv::DataType<double>::type);
            cv::Mat Y(3,1, cv::DataType<double>::type);
            pcl::PointCloud<pcl::PointXYZI>::const_iterator it;
            for(it = cloud->points.begin(); it != cloud->points.end(); it++)
            {
                if(it->x > maxX || it->x < 0.0 || abs(it->y) > maxY || it->z < minZ)
                    continue;

                X.at<double>(0,0) = it->x;
                X.at<double>(0,1) = it->y;
                X.at<double>(0,2) = it->z;
                X.at<double>(0,3) = 1;

                Y = i_params.cameraIn * i_params.RT * X;

                cv::Point pt;
                pt.x = Y.at<double>(0,0) / Y.at<double>(0,2);
                pt.y = Y.at<double>(1,0) / Y.at<double>(0,2);

                float val = it->x;
                float maxVal = 20.0;
                int red = std::min(255, (int)(255 * abs((val - maxVal) / maxVal)));
                int green = std::min(255,(int)(255 * (1 - abs((val - maxVal) / maxVal))));
                cv::circle(overlay, pt, 1, cv::Scalar(0, green, red), -1);
            }   

            ros::Time time = ros::Time::now();
            cv_ptr->encoding = "bgr8";
            cv_ptr->header.stamp = time;
            cv_ptr->header.frame_id = "veloydne";
            cv_ptr->image = overlay;
            image_publisher.publish(cv_ptr->toImageMsg());
            // std::cout<<"project picture is published!"<<std::endl;
        }
        
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "projection");
    projection pj;

    
    return 0;
}