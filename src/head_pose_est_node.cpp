#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PolygonMesh.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>

#include <Eigen/Geometry>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"

#include <iostream>
#include <stdio.h>

#define PARAM_PUB_TOPIC "pub_topic"
#define DEFAULT_PUB_TOPIC "/headPose/transform"
#define DEFAULT_PARAM_CLOUD false

using namespace cv;
using namespace std;

typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactSyncPolicy;


// publisher for head pose transformation
ros::Publisher transformPub;

string topicColor = "/kinect2/qhd/image_color_rect";
string topicDepth = "/kinect2/qhd/image_depth_rect";

string window_name = "Capture - Face detection";
// bounding rectangle of face found in last image

cv::Mat cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
cv::Mat lookupX, lookupY;

void createLookup(size_t width, size_t height)
{
    const float fx = 1.0f / cameraMatrixColor.at<double>(0, 0);
    const float fy = 1.0f / cameraMatrixColor.at<double>(1, 1);
    const float cx = cameraMatrixColor.at<double>(0, 2);
    const float cy = cameraMatrixColor.at<double>(1, 2);
    float *it;

    lookupY = cv::Mat(1, height, CV_32F);
    it = lookupY.ptr<float>();
    for(size_t r = 0; r < height; ++r, ++it)
    {
        *it = (r - cy) * fy;
    }

    lookupX = cv::Mat(1, width, CV_32F);
    it = lookupX.ptr<float>();
    for(size_t c = 0; c < width; ++c, ++it)
    {
        *it = (c - cx) * fx;
    }
}

void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image)
{
    cv_bridge::CvImageConstPtr pCvImage;
    pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
    pCvImage->image.copyTo(image);
}

void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix)
{
    double *itC = cameraMatrix.ptr<double>(0, 0);
    for(size_t i = 0; i < 9; ++i, ++itC)
    {
        *itC = cameraInfo->K[i];
    }
}

void imageCallback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
                   const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth) {

  try {
    cv::Mat image, depth;
    readImage(imageColor, image);
    readImage(imageDepth, depth);

    // the head pose processing can go here

    cv::imshow("color", image);
    cv::imshow("depth", depth);
    cv::waitKey(10);

  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", imageColor->encoding.c_str());
  }
}

int main(int argc, char **argv)
{

    // initialize ros node
    ros::init(argc, argv, "head_pose_est_node");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");

    // get console args
    std::string pubTopic;
    if (!nhPrivate.getParam(PARAM_PUB_TOPIC, pubTopic)) {
      pubTopic = DEFAULT_PUB_TOPIC;
    }

    image_transport::ImageTransport it(nh);

    // taken from kinect2_viewer to create pointcloud
    std::string topicCameraInfoColor = topicColor.substr(0, topicColor.rfind('/')) + "/camera_info";
    std::string topicCameraInfoDepth = topicDepth.substr(0, topicDepth.rfind('/')) + "/camera_info";

    std::cout << topicColor << std::endl << std::flush;
    std::cout << topicCameraInfoColor << std::endl << std::flush;
    std::cout << topicDepth << std::endl << std::flush;
    std::cout << topicCameraInfoDepth << std::endl << std::flush;

    image_transport::TransportHints hints(false ? "compressed" : "raw");
    // TODO - consider use of image_transport::CameraSubscriber do combine subscription to cameraImage and cameraInfo (create via image_transport::ImageTransport::subscribeCamera)
    // TODO - use Chain with TimeSynchronizer and TimeSequencer
    image_transport::SubscriberFilter * subImageColor = new image_transport::SubscriberFilter(it, topicColor, 1, hints);
    image_transport::SubscriberFilter * subImageDepth = new image_transport::SubscriberFilter(it, topicDepth, 1, hints);
    message_filters::Subscriber<sensor_msgs::CameraInfo> * subCameraInfoColor = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoColor, 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> * subCameraInfoDepth = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoDepth, 1);

    message_filters::Synchronizer<ExactSyncPolicy> * syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(1), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);

    transformPub = nh.advertise<std_msgs::Float64MultiArray>(pubTopic, 100);

    // setup callback
    syncExact->registerCallback(imageCallback);

    ros::spin();

}
