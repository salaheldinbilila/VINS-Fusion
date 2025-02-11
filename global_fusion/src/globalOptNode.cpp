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

/* Updates MUN ISL
   * plot the raw GPS path using custom sources
	*
   * draw the magnetic eading vector using imu message
   * align the  vins path to magnetic north after bias correction
*/

#include "ros/ros.h"
#include "globalOpt.h"
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <stdio.h>
#include <queue>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "nmea_msgs/Sentence.h"
#include <fstream>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>


GlobalOptimization globalEstimator;
ros::Publisher pub_global_odometry, pub_global_path, pub_gps_path, pub_ppk_path, pub_car;
nav_msgs::Path *global_path;
nav_msgs::Path *gps_path; // this is used to plot the gps_message path
nav_msgs::Path *ppk_path; // this is used to plot the gps_message path
map<double, vector<double>> GPSPositionMap;
double last_vio_t = -1;
std::queue<sensor_msgs::NavSatFixConstPtr> gpsQueue;
std::mutex m_buf;
//bool rtk_unreliable = false;  //TODO  : send to config file
//bool use_ppk = false;  //TODO  : send to config file
//std::string ppk_pos_file = "/media/salah/8ADC39F3DC39D9E1/bell412_dataset6/bell412_dataset6_frl.pos"; //TODO  : send to config file
std::string nmeaSentence = {};
std::string nmea_time;
boost::posix_time::time_duration nmea_time_pval;
boost::posix_time::time_duration ppk_time_pval;
boost::posix_time::ptime my_posix_time;
boost::gregorian::date my_posix_date;
boost::posix_time::time_duration my_time_of_day;


std::ifstream myfile1;
std::string line;
bool skip_read = false;
sensor_msgs::NavSatFix fix_ppk_msg;

void publish_car_model(double t, Eigen::Vector3d t_w_car, Eigen::Quaterniond q_w_car)
{
    visualization_msgs::MarkerArray markerArray_msg;
    visualization_msgs::Marker car_mesh;
    car_mesh.header.stamp = ros::Time(t);
    car_mesh.header.frame_id = "worldGPS"; //"worldGPS"
    car_mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
    car_mesh.action = visualization_msgs::Marker::ADD;
    car_mesh.id = 0;

    car_mesh.mesh_resource = "package://global_fusion/models/car.dae";

    Eigen::Matrix3d rot;
    rot << 0, 0, -1, 0, -1, 0, -1, 0, 0;
    rot << -1, 0, 0, 0, 0, -1, 0, -1, 0; //our dataset
    
    Eigen::Quaterniond Q;
    Q = q_w_car * rot; 
    car_mesh.pose.position.x    = t_w_car.x();
    car_mesh.pose.position.y    = t_w_car.y();
    car_mesh.pose.position.z    = t_w_car.z();
    car_mesh.pose.orientation.w = Q.w();
    car_mesh.pose.orientation.x = Q.x();
    car_mesh.pose.orientation.y = Q.y();
    car_mesh.pose.orientation.z = Q.z();

    car_mesh.color.a = 1.0;
    car_mesh.color.r = 1.0;
    car_mesh.color.g = 0.0;
    car_mesh.color.b = 0.0;

    float major_scale = 2.0;

    car_mesh.scale.x = major_scale;
    car_mesh.scale.y = major_scale;
    car_mesh.scale.z = major_scale;
    markerArray_msg.markers.push_back(car_mesh);
    pub_car.publish(markerArray_msg);
}

void GPS_callback(const sensor_msgs::NavSatFixConstPtr &GPS_msg)
{
    if(globalEstimator.use_ppk) return;

    m_buf.lock();
    gpsQueue.push(GPS_msg);
    m_buf.unlock();

    /*
    //1. get the lat long altitude
    double gps_t = GPS_msg->header.stamp.toSec();
    double latitude = GPS_msg->latitude;
    double longitude = GPS_msg->longitude;
    double altitude = GPS_msg->altitude;
    int fixstatus = GPS_msg->status.status;
    //2. convert to xyz
    double xyz[3];
    globalEstimator.GPS2XYZ(latitude, longitude, altitude, xyz);
    if (fixstatus==1){
    	printf("received xyz: %f,%f,%f\n", xyz[0], xyz[1], xyz[2]);
    }
    //vector<double> tmp{xyz[0], xyz[1], xyz[2], posAccuracy};
    //3. append to the gps_path
    vector<double> tmp{xyz[0], xyz[1], xyz[2], 0.0};
    GPSPositionMap[gps_t] = tmp;
    
    
    //4. publish the path
    gps_path.poses.clear();
    map<double, vector<double>>::iterator iter;
    for (iter = GPSPositionMap.begin(); iter != GPSPositionMap.end(); iter++)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time(iter->first);
        pose_stamped.header.frame_id = "worldGPS";
        pose_stamped.pose.position.x = iter->second[0];
        pose_stamped.pose.position.y = iter->second[1];
        pose_stamped.pose.position.z = iter->second[2];
        pose_stamped.pose.orientation.w = 1.0;
        pose_stamped.pose.orientation.x = 0.0;
        pose_stamped.pose.orientation.y = 0.0;
        pose_stamped.pose.orientation.z = 0.0;
        gps_path.poses.push_back(pose_stamped);
    }*/


    /*
    //printf("GPS_callback! \n");
    double t = GPS_msg->header.stamp.toSec();
    //printf("receive GPS with timestamp %f\n", GPS_msg->header.stamp.toSec());
    double latitude = GPS_msg->latitude;
    double longitude = GPS_msg->longitude;
    double altitude = GPS_msg->altitude;
    //int numSats = GPS_msg->status.service;
    double pos_accuracy = GPS_msg->position_covariance[0];
    //printf("receive covariance %lf \n", pos_accuracy);
    globalEstimator.inputGPS(t, latitude, longitude, altitude, pos_accuracy);*/

    
}

void vio_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    //printf("vio_callback! \n");
    double t = pose_msg->header.stamp.toSec();
    Eigen::Vector3d vio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
    Eigen::Quaterniond vio_q;
    vio_q.w() = pose_msg->pose.pose.orientation.w;
    vio_q.x() = pose_msg->pose.pose.orientation.x;
    vio_q.y() = pose_msg->pose.pose.orientation.y;
    vio_q.z() = pose_msg->pose.pose.orientation.z;
    globalEstimator.inputOdom(t, vio_t, vio_q);

    
    // add GPS factors to the graph
    m_buf.lock();
    while(!gpsQueue.empty())
    {
        sensor_msgs::NavSatFixConstPtr GPS_msg = gpsQueue.front();
        double gps_t = GPS_msg->header.stamp.toSec();
        printf("vio t: %f, gps t: %f \n", t, gps_t);
        // 10ms sync tolerance
        //if(gps_t >= t - 0.01 && gps_t <= t + 0.01)
        if(gps_t >= t - 0.1 && gps_t <= t + 0.1) //TODO: get from config ( this can be larger for unsynced data)  
        {
            //printf("receive GPS with timestamp %f\n", GPS_msg->header.stamp.toSec());
            double latitude = GPS_msg->latitude;
            double longitude = GPS_msg->longitude;
            double altitude = GPS_msg->altitude;
            int fixstatus = GPS_msg->status.status;  //this is 2 for rtk and 
            double pos_accuracy = GPS_msg->position_covariance[0];
            if(pos_accuracy > 0 && fixstatus >= 0){
 		if(globalEstimator.rtk_unreliable){
 			if(fixstatus == 2 ){
                	//printf("synced| receive covariance %lf | fix status(2:RTK) %i \n", pos_accuracy, fixstatus); // use this to check gps sync errors
                	globalEstimator.inputGPS(t, latitude, longitude, altitude, pos_accuracy); }}
		else {
			globalEstimator.inputGPS(t, latitude, longitude, altitude, pos_accuracy);
		}
            }
            gpsQueue.pop();
            break;
        }
        //else if(gps_t < t - 0.01)
        else if(gps_t < t - 0.1)
            gpsQueue.pop();
        //else if(gps_t > t + 0.01)
        else if(gps_t > t + 0.1)
            break;
    }
    m_buf.unlock();

    Eigen::Vector3d global_t;
    Eigen:: Quaterniond global_q;
    globalEstimator.getGlobalOdom(global_t, global_q);

    nav_msgs::Odometry odometry;
    odometry.header = pose_msg->header;
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "world";
    odometry.pose.pose.position.x = global_t.x();
    odometry.pose.pose.position.y = global_t.y();
    odometry.pose.pose.position.z = global_t.z();
    odometry.pose.pose.orientation.x = global_q.x();
    odometry.pose.pose.orientation.y = global_q.y();
    odometry.pose.pose.orientation.z = global_q.z();
    odometry.pose.pose.orientation.w = global_q.w();
    pub_global_odometry.publish(odometry);
    pub_global_path.publish(*global_path);
    pub_gps_path.publish(*gps_path);
    pub_ppk_path.publish(*ppk_path);
    publish_car_model(t, global_t, global_q);
}

void split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
}

float GpsToDecimalDegrees(const char* nmeaPos, char quadrant)
{
  float v= 0;
  if(strlen(nmeaPos)>5)
  {
    char integerPart[3+1];
    int digitCount= (nmeaPos[4]=='.' ? 2 : 3);
    memcpy(integerPart, nmeaPos, digitCount);
    integerPart[digitCount]= 0;
    nmeaPos+= digitCount;
    v= atoi(integerPart) + atof(nmeaPos)/60.;
    if(quadrant=='W' || quadrant=='S')
      v= -v;
  }
  return v;
}

ros::Time findRosTimeFromNmea(ros::Time stamp, std::string substr){
				my_posix_date   =    stamp.toBoost().date();
				//my_posix_date   =    boost::gregorian::date(2022,1,16);
	  			my_time_of_day  =    boost::posix_time::hours(std::stod(substr.substr(0,2))) +
						     boost::posix_time::minutes(std::stod(substr.substr(2,2))) + 
						     boost::posix_time::seconds(std::stod(substr.substr(4,2))) +
						     boost::posix_time::millisec(std::stod(substr.substr(7,2))*10); 
				
				my_posix_time   =    boost::posix_time::ptime(my_posix_date, my_time_of_day);
				
				

				//UTC 00:00 handling
				// get the difference in calculated clock and header clock
				boost::posix_time::time_duration time_diff = my_posix_time - stamp.toBoost();
				//ROS_INFO_STREAM("diff: "<< time_diff ) ; //ok
				//if the clock is off more than 24 hrs throw an error -this is not currently handled
				if (time_diff >= boost::posix_time::hours(24) || time_diff <= -boost::posix_time::hours(24)){
					ROS_INFO_STREAM("Clocks are off more than 24 Hrs (not handled in code) :"<< time_diff) ; //ok
					ROS_ERROR("Update code");
					ros::shutdown();
				}
				// if it is larger than 12hrs substract a day
				else if (time_diff >= boost::posix_time::hours(12)){
					my_posix_time = my_posix_time - boost::posix_time::hours(24);
					ROS_INFO_STREAM("UTC 00:00 corrected :"<< my_posix_time ) ; //ok
				}
				else if (time_diff <= -boost::posix_time::hours(12)){
					my_posix_time = my_posix_time + boost::posix_time::hours(24);
					ROS_INFO_STREAM("UTC 00:00 corrected :"<< my_posix_time ) ; //ok
					// if it is lees than -12hrs add a day
				}
			
				//perform clock correction
				return ros::Time::fromBoost(my_posix_time);

}

void nmeaCallback(const nmea_msgs::Sentence::ConstPtr& msg)
{  
    nmeaSentence = msg->sentence;
    
    //cout << nmeaSentence << endl;
    std::istringstream ss5(nmeaSentence);
    string token;
    std::getline(ss5,token, ','); string nmea_type = token;

    if (nmea_type.compare("$GNGGA") != 0)
	return;
    std::getline(ss5,token, ','); nmea_time = token;
    std::getline(ss5,token, ','); string nmea_lat_str = token; 
    if (token.size() < 2) //skip if nmea is blank
    return;
    std::getline(ss5,token, ','); double nmea_lat = GpsToDecimalDegrees(nmea_lat_str.c_str(), token.at(0));
    std::getline(ss5,token, ','); string nmea_lon_str = token; 
    std::getline(ss5,token, ','); double nmea_lon = GpsToDecimalDegrees(nmea_lon_str.c_str(), token.at(0));
    std::getline(ss5,token, ','); int nmea_fix = stoi(token);
    std::getline(ss5,token, ',');
    std::getline(ss5,token, ',');
    std::getline(ss5,token, ','); double nmea_alt = stod(token);


    std::stringstream nmea_time_concat;
    nmea_time_concat << nmea_time.substr(0,2) << ":"  << nmea_time.substr(2,2) << ":" << nmea_time.substr(4,5) << "0";
    std::string nmea_time_formatted_to_string = nmea_time_concat.str();

    
  
    //cout << "nmea received time:" << nmea_time << endl; //k
    
    //define the variables 
    std::string tempppk,templat,templon,tempalt,fix_ppk_time;
    //open file
    if (myfile1.is_open())
    {
     	
 	bool ppk_synced = false;
	while (!ppk_synced)
	{
		//0. read the next line
		if (!skip_read){
		std::getline(myfile1, line);} 
		
    		//1. remove the header info in pos file
		if(line.at(0) == '%') continue;
		//cout<<line<<endl;
 
                //2. extract the position message line
                //2022/05/25 20:43:10.200   45.325064906  -75.664896317    86.7785   1  28   0.0035   0.0025   0.0064  -0.0005  -0.0015  -0.0021   0.01    7.9
		std::string tempdate,temptime,templat,templon,tempalt,tempQ,tempnumsat,tempsdn,tempsde,tempsdu,tempsdne,tempsdeu,tempsdun,tempage,tempratio;
                std::istringstream iss1(line);
    		iss1 >> tempdate >> temptime >> templat >> templon >> tempalt >> tempQ >> tempnumsat >> tempsdn >> tempsde >> tempsdu >> tempsdne >> tempsdeu >> tempsdun >> tempage >> tempratio;
		
		//cout<<"ppk time string:"<<temptime<< " Nmea time: "<<nmea_time_formatted_to_string <<endl;//k

		// TODO:quick check of the UTC date to warn possible errors
		// convert ros time of nmea stamp to UTC and and check with PPK time
		
		// 3.1 Convert the two times to seconds
		//time value conversions
  		nmea_time_pval  =    boost::posix_time::hours(std::stod(nmea_time_formatted_to_string.substr(0,2))) +
				     boost::posix_time::minutes(std::stod(nmea_time_formatted_to_string.substr(3,2))) + 
			             boost::posix_time::seconds(std::stod(nmea_time_formatted_to_string.substr(6,2))) +
		                     boost::posix_time::millisec(std::stod(nmea_time_formatted_to_string.substr(9,2))*10);

		ppk_time_pval  =     boost::posix_time::hours(std::stod(temptime.substr(0,2))) +
				     boost::posix_time::minutes(std::stod(temptime.substr(3,2))) + 
			             boost::posix_time::seconds(std::stod(temptime.substr(6,2))-18) + //TODO:GPST to UTC correction
		                     boost::posix_time::millisec(std::stod(temptime.substr(9,2))*10);
		boost::posix_time::time_duration delta = ppk_time_pval-nmea_time_pval;
		double diff = delta.total_microseconds();
		//cout<<"ppk time val:"<< ppk_time_pval << " Nmea time val: "<< nmea_time_pval << "|" << diff << endl;//k

		//3.keep reading the file if the nmea time is ahead
                if (diff<0){
		     skip_read = false;
		     ppk_synced = false;
                     //cout<<"NMEA ahead:"<< ppk_time_pval << " Nmea time val: "<< nmea_time_pval << "|" << diff << endl;
		     continue;
		}

		//4.do not read the next line if the ppk time is ahead
		if (diff>0){
		     skip_read = true;
		     ppk_synced = true; 
		     continue;
		}

                //5.if they match input the GPS data to the optimizer
                if (diff==0){
		     skip_read = false;
		     ppk_synced = true; 


                     ros::Time nmea_ros_time=findRosTimeFromNmea(msg->header.stamp,nmea_time);
		     cout<<"** SYNCED: ppk time val:"<< ppk_time_pval << " Nmea time val: "<< nmea_time_pval << "|" << diff << "|" << nmea_lat << "|" << nmea_lon << "|" << nmea_alt << "|" <<  stof(templat) << "|" <<  stof(templon) << "|" <<  stof(tempalt) << endl;
		     cout << nmea_ros_time << "|" << msg->header.stamp << endl;
  		     // Create a ppk GPS message
	             fix_ppk_msg.header = msg->header;
		     fix_ppk_msg.status.status = 2;
		     fix_ppk_msg.status.service = 1;
	             fix_ppk_msg.latitude = stof(templat); // deg
		     fix_ppk_msg.longitude = stof(templon); // deg
		     fix_ppk_msg.altitude = stof(tempalt); // m
		     float sdn = std::stof(tempsdn)*10; // ppk covariances are too tight
	    	     float sde = std::stof(tempsde)*10;
	             float sdu = std::stof(tempsdu)*10;
                     float sdne = std::stof(tempsdne)*10;
	    	     float sdeu = std::stof(tempsdeu)*10;
	             float sdun = std::stof(tempsdun)*10;
		     fix_ppk_msg.position_covariance = {sdn, sdne, sdun, sdne, sde, sdeu, sdun, sdeu, sdu};
		     fix_ppk_msg.position_covariance_type = 3;

		     

                     // append to the ppk path and publish
                     // visualization only
                     //globalEstimator.inputPPKviz(fix_ppk_msg.header.stamp.toSec(), stof(templat), stof(templon), stof(tempalt), sdn);
                     //if(nmea_fix==5){   //5 is RTK
                     //globalEstimator.inputPPKviz(fix_ppk_msg.header.stamp.toSec(), nmea_lat, nmea_lon, nmea_alt, sdn);}
		     
                     if(globalEstimator.use_ppk) {
			// include in the GPS qextern int SAVE_GROUNDTRUTH;

                     m_buf.unlock();
		     }	
		} 
		 		
	}
    }
    else std::cout << "Unable to open file1";
    cout<<"---------------------------"<<endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "globalEstimator");
    ros::NodeHandle n("~");
    //file saving & groundtruth
    ros::param::get("/test_name", globalEstimator.test_name);
    ros::param::get("/save_groundtruth", globalEstimator.save_groundtruth);
    ros::param::get("/use_ppk", globalEstimator.use_ppk);
    ros::param::get("/rtk_unreliable", globalEstimator.rtk_unreliable);
    globalEstimator.algo_file.open(globalEstimator.test_name + ".txt", std::ios::trunc);
    if(globalEstimator.save_groundtruth)
    {
        globalEstimator.gps_filename = globalEstimator.test_name + "_gps";
        globalEstimator.gt_filename = globalEstimator.test_name + "_gt";
        if (globalEstimator.use_ppk)
        {
            globalEstimator.gps_filename = globalEstimator.gps_filename + "_ppk";
            globalEstimator.gt_filename = globalEstimator.gt_filename + "_ppk";
        }
        globalEstimator.gps_file.open(globalEstimator.gps_filename + ".txt", std::ios::trunc);
    }
    /*
    if(globalEstimator.use_ppk)
    {
        ros::param::get("/ppk_pos_file", globalEstimator.ppk_pos_file);
        myfile1.open(globalEstimator.ppk_pos_file);
    }
    */

    global_path = &globalEstimator.global_path;
    gps_path = &globalEstimator.gps_path;
    ppk_path = &globalEstimator.ppk_path;


    ros::Subscriber sub_nmea = n.subscribe("/nmea_sentence", 100, nmeaCallback);
    ros::Subscriber sub_GPS = n.subscribe("/fix", 100, GPS_callback);
    ros::Subscriber sub_vio = n.subscribe("/vins_estimator/odometry", 100, vio_callback);
    pub_global_path = n.advertise<nav_msgs::Path>("global_path", 100);
    pub_gps_path = n.advertise<nav_msgs::Path>("gps_path", 100);
    pub_ppk_path = n.advertise<nav_msgs::Path>("ppk_path", 100);
    pub_global_odometry = n.advertise<nav_msgs::Odometry>("global_odometry", 100);
    pub_car = n.advertise<visualization_msgs::MarkerArray>("car_model", 100);
    ros::spin();
    return 0;
}
