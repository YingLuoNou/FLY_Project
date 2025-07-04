# 修改保存海康相机源码
```cpp
#include <iostream>
#include "opencv2/opencv.hpp"
#include <vector>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include "hikrobot_camera.hpp"

// 剪裁掉照片和雷达没有重合的视角，去除多余像素可以使rosbag包变小
#define FIT_LIDAR_CUT_IMAGE false
#if FIT_LIDAR_CUT_IMAGE
    #define FIT_min_x 420
    #define FIT_min_y 70
    #define FIT_max_x 2450
    #define FIT_max_y 2000
#endif 

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    //********** variables    **********/
    cv::Mat src;

    //********** rosnode init **********/
    ros::init(argc, argv, "hikrobot_camera");
    ros::NodeHandle hikrobot_camera;
    camera::Camera MVS_cap(hikrobot_camera);

    image_transport::ImageTransport main_cam_image(hikrobot_camera);
    image_transport::CameraPublisher image_pub = main_cam_image.advertiseCamera("/hikrobot_camera/rgb", 1000);

    sensor_msgs::Image image_msg;
    sensor_msgs::CameraInfo camera_info_msg;
    cv_bridge::CvImagePtr cv_ptr = boost::make_shared<cv_bridge::CvImage>();
    cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;

    // 设置内参,修改成你的相机内参
    camera_info_msg.distortion_model = "plumb_bob";
    camera_info_msg.D = {-0.06955923178188053, 0.07234046131495264, -1.214292289261009e-05, -1.4166903141799492e-05, 0.0};
    camera_info_msg.K = {1269.097262536443, 0.0, 617.2720977389938,
                         0.0, 1269.0172395858178, 530.7033173063891,
                         0.0, 0.0, 1.0};
    camera_info_msg.R = {1.0, 0.0, 0.0,
                         0.0, 1.0, 0.0,
                         0.0, 0.0, 1.0};
    camera_info_msg.P = {1251.305908203125, 0.0, 616.3885840335788, 0.0,
                         0.0, 1255.6256103515625, 530.4659520190944, 0.0,
                         0.0, 0.0, 1.0, 0.0};

    // 设置图像尺寸（根据实际图像大小）
    camera_info_msg.width = 1280;
    camera_info_msg.height = 1024;

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();

        MVS_cap.ReadImg(src);
        if (src.empty())
        {
            continue;
        }

#if FIT_LIDAR_CUT_IMAGE
        cv::Rect area(FIT_min_x, FIT_min_y, FIT_max_x - FIT_min_x, FIT_max_y - FIT_min_y);
        cv::Mat src_new = src(area);
        cv_ptr->image = src_new;
#else
        cv_ptr->image = src;
#endif

        image_msg = *(cv_ptr->toImageMsg());
        image_msg.header.stamp = ros::Time::now();
        image_msg.header.frame_id = "hikrobot_camera";

        camera_info_msg.header.frame_id = image_msg.header.frame_id;
        camera_info_msg.header.stamp = image_msg.header.stamp;

        image_pub.publish(image_msg, camera_info_msg);
    }

    return 0;
}
```

# 旧版请修改.cpp+.h文件
hk_camera_node.cpp
```hk_camera_node.cpp
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>    // 新增
#include <std_srvs/Empty.h>

#include "hk_camera.h"
#include "hk_camera_node.h"

HkCamNode::~HkCamNode()
{
    cam_.shutdown();
}

HkCamNode::HkCamNode()
  : node_("~")
{
    image_transport::ImageTransport image_topic(node_);
    image_pub_ = image_topic.advertiseCamera("image_raw", 1);

    // --- 初始化 CameraInfo ---
    camera_info_msg_.header.frame_id = "hk_camera";  // 可通过参数覆盖
    camera_info_msg_.width  = cam_.stConvertParam.nWidth;   // 或者硬编码 1280
    camera_info_msg_.height = cam_.stConvertParam.nHeight;  // 或者硬编码 1024
    camera_info_msg_.distortion_model = "plumb_bob";
    camera_info_msg_.D = {
        -0.12311137126827945,
         0.15063967043439314,
         0.005907959586455938,
         0.004277916769468728,
         0.0
    };
    camera_info_msg_.K = {
        1240.1576739187901, 0.0,               626.651182091468,
           0.0,           1238.1367057015384, 543.8797945124209,
           0.0,            0.0,               1.0
    };
    camera_info_msg_.R = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    };
    camera_info_msg_.P = {
        1214.147705078125, 0.0,                631.3703164561739, 0.0,
           0.0,           1214.4398193359375, 548.1321324880882, 0.0,
           0.0,             0.0,                1.0,                0.0
    };
    // --- 参数和服务同原来 ---
    node_.param("camera_frame_id", camera_info_msg_.header.frame_id, camera_info_msg_.header.frame_id);
    service_start_ = node_.advertiseService("start_capture", &HkCamNode::service_start_cap, this);
    service_stop_  = node_.advertiseService("stop_capture",  &HkCamNode::service_stop_cap,  this);

    ROS_INFO("Starting HkCamNode");
    cam_.start();
}

bool HkCamNode::send_image()
{
    if (cam_.grab_rgb_image())
    {
        // 填充 Image
        img_msg.header.stamp = ros::Time::now();
        fillImage(img_msg,
                  "rgb8",
                  cam_.stConvertParam.nHeight,
                  cam_.stConvertParam.nWidth,
                  3 * cam_.stConvertParam.nWidth,
                  cam_.stConvertParam.pDstBuffer);

        // 更新并发布 CameraInfo
        camera_info_msg_.header.stamp = img_msg.header.stamp;
        image_pub_.publish(img_msg, camera_info_msg_);
        return true;
    }
    return false;
}

bool HkCamNode::spin()
{
    ros::Rate loop_rate(10);
    while (node_.ok())
    {
        if (!send_image())
            ROS_WARN("HIKROBOT camera did not respond in time.");
        ros::spinOnce();
        loop_rate.sleep();
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hk_camera_node");
    HkCamNode a;
    a.spin();
    return EXIT_SUCCESS;
}
bool HkCamNode::service_start_cap(std_srvs::Empty::Request &req,
                                  std_srvs::Empty::Response &res)
{
    ROS_INFO("Received start_capture service call");
    cam_.start();
    return true;
}

bool HkCamNode::service_stop_cap(std_srvs::Empty::Request &req,
                                 std_srvs::Empty::Response &res)
{
    ROS_INFO("Received stop_capture service call");
    cam_.shutdown();
    return true;
}
```  
hk_camera_node.h
```h
#include <sensor_msgs/CameraInfo.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <std_srvs/Empty.h>

#include "hk_camera.h"

#include "MvCameraControl.h"

class HkCamNode
{
public:
        ros::NodeHandle node_;
        ros::ServiceServer service_start_, service_stop_;
        sensor_msgs::Image img_msg;
        sensor_msgs::CameraInfo    camera_info_msg_;
        image_transport::CameraPublisher image_pub_;
        HkCam cam_;

        HkCamNode();
        ~HkCamNode();
        bool service_start_cap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
        bool service_stop_cap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

        bool send_image();
        bool spin();
};
```
