/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"

#define time_out_count 25

Estimator estimator;

queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
queue<sensor_msgs::ImageConstPtr> seg_buf;
queue<sensor_msgs::ImageConstPtr> det_buf;
std::mutex mtx_lidar;
std::mutex m_buf;
std::mutex m_odom;

// global variable for saving the depthCloud shared between two threads
pcl::PointCloud<PointType>::Ptr depthCloud(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr depthCloudLocal(new pcl::PointCloud<PointType>());

// global variables saving the lidar point cloud
deque<pcl::PointCloud<PointType>> cloudQueue;
deque<double> timeQueue;

// global variable saving the lidar odometry
deque<nav_msgs::Odometry> odomQueue;
odometryRegister *odomRegister;

void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    m_odom.lock();
    odomQueue.push_back(*odom_msg);
    m_odom.unlock();
}

void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img0_buf.push(img_msg);
    m_buf.unlock();
}

void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img1_buf.push(img_msg);
    m_buf.unlock();
}

// Segmentation callback
void seg_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    seg_buf.push(img_msg);
    //ROS_INFO("seg callback successful");
    m_buf.unlock();
}

// Detection callback
void det_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    det_buf.push(img_msg);
    //ROS_INFO("det callback successful");
    m_buf.unlock();
}


cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}

void lidar_callback(const sensor_msgs::PointCloud2ConstPtr& laser_msg)
{
    static int lidar_count = -1;
    if (++lidar_count % (LIDAR_SKIP+1) != 0)
        return;

    // 0. listen to transform
    static tf::TransformListener listener;
    static tf::StampedTransform transform;
    try{
        listener.waitForTransform("world", "body_ros", laser_msg->header.stamp, ros::Duration(0.01));
        listener.lookupTransform("world", "body_ros", laser_msg->header.stamp, transform);
    } 
    catch (tf::TransformException ex){
        // ROS_ERROR("lidar no tf");
        return;
    }

    double xCur, yCur, zCur, rollCur, pitchCur, yawCur;
    xCur = transform.getOrigin().x();
    yCur = transform.getOrigin().y();
    zCur = transform.getOrigin().z();
    tf::Matrix3x3 m(transform.getRotation());
    m.getRPY(rollCur, pitchCur, yawCur);
    Eigen::Affine3f transNow = pcl::getTransformation(xCur, yCur, zCur, rollCur, pitchCur, yawCur);

    // 1. convert laser cloud message to pcl
    pcl::PointCloud<PointType>::Ptr laser_cloud_in(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*laser_msg, *laser_cloud_in);

    // 2. downsample new cloud (save memory)
    pcl::PointCloud<PointType>::Ptr laser_cloud_in_ds(new pcl::PointCloud<PointType>());
    static pcl::VoxelGrid<PointType> downSizeFilter;
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilter.setInputCloud(laser_cloud_in);
    downSizeFilter.filter(*laser_cloud_in_ds);
    *laser_cloud_in = *laser_cloud_in_ds;

    // 3. filter lidar points (only keep points in camera view)
    pcl::PointCloud<PointType>::Ptr laser_cloud_in_filter(new pcl::PointCloud<PointType>());
    for (int i = 0; i < (int)laser_cloud_in->size(); ++i)
    {
        PointType p = laser_cloud_in->points[i];
        if (p.x >= 0 && abs(p.y / p.x) <= 10 && abs(p.z / p.x) <= 10)
            laser_cloud_in_filter->push_back(p);
    }
    *laser_cloud_in = *laser_cloud_in_filter;

    // TODO: transform to IMU body frame
    // 4. offset T_lidar -> T_camera 
    pcl::PointCloud<PointType>::Ptr laser_cloud_offset(new pcl::PointCloud<PointType>());
    Eigen::Affine3f transOffset = pcl::getTransformation(L_C_TX, L_C_TY, L_C_TZ, L_C_RX, L_C_RY, L_C_RZ);
    pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_offset, transOffset);
    *laser_cloud_in = *laser_cloud_offset;

    // 5. transform new cloud into global odom frame
    pcl::PointCloud<PointType>::Ptr laser_cloud_global(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_global, transNow);

    // 6. save new cloud
    double timeScanCur = laser_msg->header.stamp.toSec();
    cloudQueue.push_back(*laser_cloud_global);
    timeQueue.push_back(timeScanCur);
    *depthCloudLocal = *laser_cloud_in;

    // 7. pop old cloud
    while (!timeQueue.empty())
    {
        if (timeScanCur - timeQueue.front() > 5.0)
        {
            cloudQueue.pop_front();
            timeQueue.pop_front();
        } else {
            break;
        }
    }
    
    std::lock_guard<std::mutex> lock(mtx_lidar);
    
    // 8. fuse global cloud
    depthCloud->clear();
    if (USE_DENSE_CLOUD == 0){
    	*depthCloud += cloudQueue.back();
    }else {
    	for (int i = 0; i < (int)cloudQueue.size(); ++i)
        	*depthCloud += cloudQueue[i];
    }

    // 9. downsample global cloud
    pcl::PointCloud<PointType>::Ptr depthCloudDS(new pcl::PointCloud<PointType>());
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilter.setInputCloud(depthCloud);
    downSizeFilter.filter(*depthCloudDS);
    *depthCloud = *depthCloudDS;
}

// extract images with same timestamp from two topics
void sync_process()
{
    while(1)
    {
        if(STEREO)
        {
            cv::Mat image0, image1;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if (!img0_buf.empty() && !img1_buf.empty())
            {
                double time0 = img0_buf.front()->header.stamp.toSec();
                double time1 = img1_buf.front()->header.stamp.toSec();
                // 0.003s sync tolerance
                if(time0 < time1 - 0.003)
                {
                    img0_buf.pop();
                    printf("throw img0\n");
                }
                else if(time0 > time1 + 0.003)
                {
                    img1_buf.pop();
                    printf("throw img1\n");
                }
                else
                {
                    time = img0_buf.front()->header.stamp.toSec();
                    header = img0_buf.front()->header;
                    image0 = getImageFromMsg(img0_buf.front());
                    img0_buf.pop();
                    image1 = getImageFromMsg(img1_buf.front());
                    img1_buf.pop();
                    //printf("find img0 and img1\n");
                }
            }
            m_buf.unlock();
            if(!image0.empty())
                estimator.inputImage(time, image0, image1);
        }
        else
        {
            cv::Mat image,seg_im,det_im;
            std_msgs::Header header,seg_header,det_header;
            double time = 0 ,seg_time = 0 ,det_time = 0;
            static char empty_count = 0;
            m_buf.lock();
            // check if the segmentation buffer is filled instead of the image buffer
            if(!img0_buf.empty())
            {
                time = img0_buf.front()->header.stamp.toSec();
                header = img0_buf.front()->header;
                //ROS_INFO("Image header: %f",time);
                if (SEG)
                {
                    if(!seg_buf.empty())
                    {
                        seg_time = seg_buf.front()->header.stamp.toSec();
                        seg_header = seg_buf.front()->header;
                        ROS_INFO("segmentation time: %f",seg_time);
                        ROS_INFO("segmentation time diff: %f",seg_time-time);
                        if (seg_time < time)
                        {
                            ROS_INFO("The segmentation is of an old image");
                            seg_buf.pop();
                        }
                        else if (seg_time > time)
                        {
                            ROS_INFO("The image is not segmented, discard");
                            img0_buf.pop();
                        }
                        else if (seg_time == time)
                        {
                            ROS_INFO("Images match");
                            image = getImageFromMsg(img0_buf.front());
                            img0_buf.pop();
                            seg_im = getImageFromMsg(seg_buf.front());
                            seg_buf.pop();
                        }
                    }
                }
                else if (DET)
                {
                    if(!det_buf.empty())
                    {
                        empty_count = 0;
                        det_time = det_buf.front()->header.stamp.toSec();
                        //ROS_INFO("detection time: %f",det_time);
                        //ROS_INFO("detection time diff: %f",det_time-time);
                        if (det_time < time)
                        {
                            //ROS_INFO("The detection is of an old image, discard");
                            det_buf.pop();
                        }
                        else if (det_time > time)
                        {
                            //ROS_INFO("Image too late. Use image without detection");
                            image = getImageFromMsg(img0_buf.front());
                            img0_buf.pop();
                        }
                        else //(det_time == time)
                        {
                            //ROS_INFO("Images match, process both");
                            image = getImageFromMsg(img0_buf.front());
                            img0_buf.pop();
                            estimator.featureTracker.det_img = getImageFromMsg(det_buf.front());
                            det_buf.pop();
                        }
                    }
                    else
                    {
                        empty_count++;
                        ROS_INFO("Detection buffer empty, count: %d",empty_count);
                        if (empty_count == time_out_count)
                        {
                            empty_count = 0;
                            ROS_INFO("Buffer still empty. Use image without detection");
                            image = getImageFromMsg(img0_buf.front());
                            img0_buf.pop();
                        }
                    }
                }
                else
                {
                    image = getImageFromMsg(img0_buf.front());
                    img0_buf.pop();
                }
            }
            m_buf.unlock();
            if (SEG)
            {
                if(!seg_im.empty())
                {
                    estimator.featureTracker.seg_img = seg_im;
                    //seg_im.release();
                } 
            }
            /*
            else if (DET)
            {
                if(!det_im.empty())
                {
                    estimator.det_img = det_im;
                }
            }
            */
            if(!image.empty())
            {
                mtx_lidar.lock();
                *estimator.depth_cloud = *depthCloud;
                mtx_lidar.unlock();
                // Get initialization info from lidar odometry
                m_odom.lock();
                estimator.initialization_info = odomRegister->getOdometry(odomQueue, time + estimator.td);
                m_odom.unlock();
                estimator.inputImage(time, image);
            }
        }

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Vector3d acc(dx, dy, dz);
    Vector3d gyr(rx, ry, rz);
    estimator.inputIMU(t, acc, gyr);
    return;
}


void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> featureFrame;
    for (unsigned int i = 0; i < feature_msg->points.size(); i++)
    {
        int feature_id = feature_msg->channels[0].values[i];
        int camera_id = feature_msg->channels[1].values[i];
        double x = feature_msg->points[i].x;
        double y = feature_msg->points[i].y;
        double z = feature_msg->points[i].z;
        double p_u = feature_msg->channels[2].values[i];
        double p_v = feature_msg->channels[3].values[i];
        double velocity_x = feature_msg->channels[4].values[i];
        double velocity_y = feature_msg->channels[5].values[i];
        double depth = -1;
        if(feature_msg->channels.size() > 5)
        {
            double gx = feature_msg->channels[6].values[i];
            double gy = feature_msg->channels[7].values[i];
            double gz = feature_msg->channels[8].values[i];
            pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
            //printf("receive pts gt %d %f %f %f\n", feature_id, gx, gy, gz);
        }
        if(feature_msg->channels.size() > 9)
            depth = feature_msg->channels[9].values[i];
        ROS_ASSERT(z == 1);
        Eigen::Matrix<double, 8, 1> xyz_uv_velocity_depth;
        xyz_uv_velocity_depth << x, y, z, p_u, p_v, velocity_x, velocity_y, depth;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity_depth);
    }
    double t = feature_msg->header.stamp.toSec();
    estimator.inputFeature(t, featureFrame);
    return;
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        estimator.clearState();
        estimator.setParameter();
    }
    return;
}

void imu_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use IMU!");
        estimator.changeSensorType(1, STEREO);
    }
    else
    {
        //ROS_WARN("disable IMU!");
        estimator.changeSensorType(0, STEREO);
    }
    return;
}

void cam_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use stereo!");
        estimator.changeSensorType(USE_IMU, 1);
    }
    else
    {
        //ROS_WARN("use mono camera (left)!");
        estimator.changeSensorType(USE_IMU, 0);
    }
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    if(argc != 2)
    {
        printf("please intput: rosrun vins vins_node [config file] \n"
               "for example: rosrun vins vins_node "
               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 1;
    }

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);

    readParameters(config_file);
    estimator.setParameter();
    estimator.featureTracker.car_mask = cv::imread(CAR_MASK, 0);
    estimator.featureTracker.bus_mask = cv::imread(BUS_MASK, 0);
    //file saving & groundtruth
    ros::param::set("/test_name", TEST_NAME);
    ros::param::set("/use_ppk", USE_PPK);
    ros::param::set("/save_groundtruth", SAVE_GROUNDTRUTH);
    ros::param::set("/rtk_unreliable", RTK_UNRELIABLE);
    ros::param::set("/ppk_pos_file", PPK_POS_FILE);

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    ROS_WARN("waiting for image and imu...");

    registerPub(n);
    estimator.depthRegister = new DepthRegister(n);
    odomRegister = new odometryRegister(n);

    ros::Subscriber sub_imu;
    if(USE_IMU)
        sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_feature = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    ros::Subscriber sub_img0 = n.subscribe(IMAGE0_TOPIC, 100, img0_callback);
    ros::Subscriber sub_img1;
    if(STEREO)
        sub_img1 = n.subscribe(IMAGE1_TOPIC, 100, img1_callback);
    ros::Subscriber sub_seg;
    if(SEG)
        sub_seg = n.subscribe("/semantic", 100, seg_callback);      // segmentation subscriber
    ros::Subscriber sub_det;
    if(DET)
        sub_det = n.subscribe("/detect", 100, det_callback);      // detection subscriber
    //ros::Subscriber sub_odom;
    ros::Subscriber sub_lidar;
    if (USE_LIDAR)
    {
        sub_lidar = n.subscribe(POINT_CLOUD_TOPIC, 100,    lidar_callback);
        //sub_odom = n.subscribe("odometry/imu", 5000, odom_callback);
    }
    ros::Subscriber sub_restart = n.subscribe("/vins_restart", 100, restart_callback);
    ros::Subscriber sub_imu_switch = n.subscribe("/vins_imu_switch", 100, imu_switch_callback);
    ros::Subscriber sub_cam_switch = n.subscribe("/vins_cam_switch", 100, cam_switch_callback);

    std::thread sync_thread{sync_process};
    //ros::spin();
    // four ROS spinners for parallel processing (segmentation, detection, image, lidar and lidar odometry)
    ros::MultiThreadedSpinner spinner(7);
    spinner.spin();

    return 0;
}
