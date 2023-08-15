#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <stereo_msgs/DisparityImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>

ros::Publisher pointcloud_pub;
sensor_msgs::CameraInfo::ConstPtr ros_cam_info;
constexpr float baseline = 64e-3; // 64 mm, from ZED-M datasheet


void camera_info_callback(const sensor_msgs::CameraInfo::ConstPtr &msg)
{
    ros_cam_info = msg;
}

void registered_callback(const sensor_msgs::ImageConstPtr &rgb_msg, const stereo_msgs::DisparityImageConstPtr &disp_msg)
{
    cv_bridge::CvImagePtr cv_ptr_rgb, cv_ptr_disp;

    try
    {
        cv_ptr_rgb = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
        cv_ptr_disp = cv_bridge::toCvCopy(disp_msg->image, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }


    float fx = ros_cam_info->P[0];
    float cx = ros_cam_info->P[2];
    float cy = ros_cam_info->P[6];
    float T = baseline;

    cv::Mat Q = (cv::Mat_<float>(4, 4) << 1, 0, 0, -cx, 0, 1, 0, -cy, 0, 0, 0, fx, 0, 0, -1 / T, 0);
    cv::Mat points3D;
    cv::reprojectImageTo3D(cv_ptr_disp->image, points3D, Q);

    // Find the shape of points3D
    int rows = points3D.rows;
    int cols = points3D.cols;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pointcloud->width = rows;
    pointcloud->height = cols;
    pointcloud->is_dense = false;
    pointcloud->points.resize(rows * cols);

    for(int i = 0; i < rows; ++i)
    {
        for(int j = 0; j < cols; ++j)
        {
            cv::Point3f point = points3D.at<cv::Point3f>(i, j);
            cv::Vec3b color = cv_ptr_rgb->image.at<cv::Vec3b>(i, j);

            // Changing this convention so that it is consistent with the ZED convention
            pcl::PointXYZRGB point_xyzrgb;
            point_xyzrgb.x = point.z;
            point_xyzrgb.y = -point.x;
            point_xyzrgb.z = -point.y;
            point_xyzrgb.r = color[2];
            point_xyzrgb.g = color[1];
            point_xyzrgb.b = color[0];

            pointcloud->points[i * cols + j] = point_xyzrgb;
        }
    }

    sensor_msgs::PointCloud2 pointcloud_msg;
    pcl::toROSMsg(*pointcloud, pointcloud_msg);

    pointcloud_msg.header = rgb_msg->header;
    pointcloud_msg.header.frame_id = "zedm_left_camera_frame";

    pointcloud_pub.publish(pointcloud_msg);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "depth_projection");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/zedm/zed_node/left_raw/image_raw_color", 1);
    message_filters::Subscriber<stereo_msgs::DisparityImage> disp_sub(nh, "/zedm/zed_node/disparity/disparity_image", 1);
    ros::Subscriber ros_cam_info_sub = nh.subscribe("/zedm/zed_node/left/camera_info", 1, camera_info_callback);

    pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud/registered_projection", 10);

    message_filters::TimeSynchronizer<sensor_msgs::Image, stereo_msgs::DisparityImage> sync(rgb_sub, disp_sub, 10);
    sync.registerCallback(boost::bind(&registered_callback, _1, _2));

    ros::spin();

    return 0;
}
