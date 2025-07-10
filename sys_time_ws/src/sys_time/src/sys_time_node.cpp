#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include "livox_ros_driver2/CustomMsg.h"  // 引入Livox自定义消息头文件

class subscriberANDpublisher {
public:
    subscriberANDpublisher();  // 构造函数声明

    void callback(const sensor_msgs::ImuConstPtr& imu_msg,
                  const livox_ros_driver2::CustomMsgConstPtr& lidar_msg,  // 修改为CustomMsg类型
                  const sensor_msgs::ImageConstPtr& image_msg);  // 回调函数声明

private:
    ros::NodeHandle nh;  // ROS节点句柄
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub;  // IMU订阅器
    message_filters::Subscriber<livox_ros_driver2::CustomMsg> lidar_sub;  // 修改为Livox自定义消息订阅器
    message_filters::Subscriber<sensor_msgs::Image> camera_sub;  // 相机订阅器

    // 同步处理器，更新同步策略，只同步 IMU、LiDAR 和相机图像数据
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, 
                                                            livox_ros_driver2::CustomMsg,  // 修改为Livox自定义消息
                                                            sensor_msgs::Image> syncpolicy;
    boost::shared_ptr<message_filters::Synchronizer<syncpolicy>> sync_;

    // 发布器
    ros::Publisher camera_pub;
    ros::Publisher lidar_pub;
    ros::Publisher imu_pub;
};

// 构造函数实现
subscriberANDpublisher::subscriberANDpublisher()
{
    // 订阅话题
    imu_sub.subscribe(nh, "/livox/imu", 10);  // 订阅IMU数据
    lidar_sub.subscribe(nh, "/livox/lidar", 10);  // 订阅Livox自定义消息类型的LiDAR点云数据
    camera_sub.subscribe(nh, "/hikrobot_camera/rgb", 10);  // 订阅相机图像数据

    // 消息过滤器，使用 ApproximateTime 进行时间同步（允许一定程度的时间误差）
    sync_.reset(new message_filters::Synchronizer<syncpolicy>(syncpolicy(10), imu_sub, lidar_sub, camera_sub));
    sync_->registerCallback(boost::bind(&subscriberANDpublisher::callback, this, _1, _2, _3));

    // 发布器
    camera_pub = nh.advertise<sensor_msgs::Image>("sync/camera_image", 10);
    lidar_pub = nh.advertise<livox_ros_driver2::CustomMsg>("sync/lidar", 10);  // 修改为Livox自定义消息类型的发布器
    imu_pub = nh.advertise<sensor_msgs::Imu>("sync/imu", 10);
}

// 回调函数
void subscriberANDpublisher::callback(const sensor_msgs::ImuConstPtr& imu_msg,
                                      const livox_ros_driver2::CustomMsgConstPtr& lidar_msg,  // 修改为Livox自定义消息类型
                                      const sensor_msgs::ImageConstPtr& image_msg) 
{
    ROS_INFO("Received synchronized message!");

    // 获取时间戳
    ros::Time imu_time = imu_msg->header.stamp;
    ros::Time lidar_time = lidar_msg->header.stamp;
    ros::Time image_time = image_msg->header.stamp;

    // 计算时间差
    ros::Duration imu_lidar_diff = imu_time - lidar_time;
    ros::Duration imu_image_diff = imu_time - image_time;
    ros::Duration lidar_image_diff = lidar_time - image_time;

    // 输出时间误差（同步误差）
    ROS_INFO("Time difference between IMU and LiDAR: %f seconds", imu_lidar_diff.toSec());
    ROS_INFO("Time difference between IMU and Image: %f seconds", imu_image_diff.toSec());
    ROS_INFO("Time difference between LiDAR and Image: %f seconds", lidar_image_diff.toSec());

    // 为每个消息设置当前时间戳（如果需要）
    ros::Time timestamp = ros::Time::now();  // 获取当前ROS时间
    sensor_msgs::Image image_msg_with_timestamp = *image_msg;
    image_msg_with_timestamp.header.stamp = timestamp;  // 设置时间戳
    livox_ros_driver2::CustomMsg lidar_msg_with_timestamp = *lidar_msg;
    lidar_msg_with_timestamp.header.stamp = timestamp;  // 设置时间戳
    sensor_msgs::Imu imu_msg_with_timestamp = *imu_msg;
    imu_msg_with_timestamp.header.stamp = timestamp;  // 设置时间戳

    // 将同步的消息发布到新的话题
    camera_pub.publish(image_msg_with_timestamp);
    lidar_pub.publish(lidar_msg_with_timestamp);
    imu_pub.publish(imu_msg_with_timestamp);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "subscriber_and_publisher_sync");  // ROS节点初始化

    // 创建对象，自动调用构造函数
    subscriberANDpublisher sp;
    ROS_INFO("begin!");

    ros::spin();  // 保持节点运行，等待回调
    return 0;
}
