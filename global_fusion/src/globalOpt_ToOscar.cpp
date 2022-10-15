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

#include "globalOpt.h"
#include "Factors.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

//Begin - add counter variables
int optcounter = 0;
int gpsPosCounter = 0;
int vinsPosCounter = 0;
int updateGlobalPathCount = 0;
//end - add counter variables


GlobalOptimization::GlobalOptimization():
outfileOdom("resultsOdom.txt", std::ios_base::trunc),
outfileGt("resultsGt.txt", std::ios_base::trunc),
//1. begin - Add files here
outfileVINS("VINS_bell412_dataset6_ppk.txt", std::ios_base::trunc),
outfileGPS("GPS_bell412_dataset6_ppk.txt", std::ios_base::trunc),
outfileFusion("Fusion_bell412_dataset6_ppk.txt", std::ios_base::trunc)
//1. end - add text files

{
    initGPS = false;
    newGPS = false;
    WGPS_T_WVIO = Eigen::Matrix4d::Identity(); //update this to the difference
    WGPS_T_WVIO_viz = Eigen::Matrix4d::Identity();
    update_count =0;
    GTframeCount = 0;
    threadOpt = std::thread(&GlobalOptimization::optimize, this);
}

GlobalOptimization::~GlobalOptimization()
{
    threadOpt.detach();
}

void GlobalOptimization::GPS2XYZ(double latitude, double longitude, double altitude, double* xyz)
{
    if(!initGPS)
    {
        geoConverter.Reset(latitude, longitude, altitude);
        initGPS = true;
    }
    geoConverter.Forward(latitude, longitude, altitude, xyz[0], xyz[1], xyz[2]);
    printf("la: %f lo: %f al: %f\n", latitude, longitude, altitude);
    printf("gps x: %f y: %f z: %f\n", xyz[0], xyz[1], xyz[2]);
}

void GlobalOptimization::inputOdom(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ)
{
	mPoseMap.lock();
    vector<double> localPose{OdomP.x(), OdomP.y(), OdomP.z(), 
    					     OdomQ.w(), OdomQ.x(), OdomQ.y(), OdomQ.z()};
    localPoseMap[t] = localPose;

    //2. Begin - VINS CSV wrighting
    Eigen::Matrix3d odomR = OdomQ.normalized().toRotationMatrix();
    std::ofstream foutE("VINS_bell412_dataset1_ppk.txt", std::ios::app); 
    vinsPosCounter++;
    foutE.setf(std::ios::fixed, std::ios::floatfield);
    foutE.precision(0);
    foutE << vinsPosCounter << " ";
    foutE.precision(9);
    foutE << t  << " "
           << odomR(0,0) << " "
           << odomR(0,1) << " "
           << odomR(0,2) << " "
            << OdomP.x()  << " "
            << odomR(1,0) << " "
            << odomR(1,1) << " "
            << odomR(1,2) << " "
            << OdomP.y()  << " "
            << odomR(2,0) << " "
            << odomR(2,1) << " "
            << odomR(2,2) << " "
            << OdomP.z()  << std::endl;
    //2. End - VINS CSV wrighting

    //rav begin
    //add the static transformation here - WGPS_T_WVIO - rav
    Eigen::Matrix4d initRotAfterOpti;
    initRotAfterOpti <<  0.495238,   0.865899,    0.0704165,  5.9154,
                         0.867781,  -0.496895,    0.00713753, 2.29296,
                         0.04117,    0.0575713,  -0.997492,   2.68105,
                         0,          0,           0,          1;
    
    Eigen::Quaterniond initglobalQ;
    initglobalQ = initRotAfterOpti.block<3, 3>(0, 0);
    Eigen::Vector3d initglobalP = initRotAfterOpti.block<3, 1>(0, 3);

    //Eigen::Matrix4d rotateRlobal;
    //rotateRlobal = initRotAfterOpti*WGPS_T_WVIO;

    //Eigen::Quaterniond globalQ;
    //globalQ = rotateRlobal.block<3, 3>(0, 0) * OdomQ;
    //Eigen::Vector3d globalP = rotateRlobal.block<3, 3>(0, 0) * OdomP + rotateRlobal.block<3, 1>(0, 3);

    Eigen::Quaterniond globalQ;
    globalQ = WGPS_T_WVIO.block<3, 3>(0, 0) * OdomQ;
    Eigen::Vector3d globalP = WGPS_T_WVIO.block<3, 3>(0, 0) * OdomP + WGPS_T_WVIO.block<3, 1>(0, 3);

    //save on its local pose

    //rav end 

    //Eigen::Quaterniond globalQ;
    //globalQ = WGPS_T_WVIO.block<3, 3>(0, 0) * OdomQ;
    //Eigen::Vector3d globalP = WGPS_T_WVIO.block<3, 3>(0, 0) * OdomP + WGPS_T_WVIO.block<3, 1>(0, 3);

    vector<double> globalPose{globalP.x(), globalP.y(), globalP.z(),
                              globalQ.w(), globalQ.x(), globalQ.y(), globalQ.z()};
    globalPoseMap[t] = globalPose;
    lastP = globalP;
    lastQ = globalQ;

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time(t);
    pose_stamped.header.frame_id = "worldGPS";
    pose_stamped.pose.position.x = lastP.x();
    pose_stamped.pose.position.y = lastP.y();
    pose_stamped.pose.position.z = lastP.z();
    pose_stamped.pose.orientation.x = lastQ.x();
    pose_stamped.pose.orientation.y = lastQ.y();
    pose_stamped.pose.orientation.z = lastQ.z();
    pose_stamped.pose.orientation.w = lastQ.w();
    global_path.header = pose_stamped.header;
    global_path.poses.push_back(pose_stamped);

    //std::cout << "Initial WGPS_T_WVIO check:" << '\n' << globalQ.matrix() << '\n'; //rav


    //Publish the worldGPS frame (only perform 100 updates and stop)
    if (1){
    //if (update_count <300){
        WGPS_T_WVIO_viz = WGPS_T_WVIO; 
    	update_count++;
        if (update_count ==300)
          printf("*********************WGPS_T_WVIO_viz fixed*********************\n");
    }
 
    static tf2_ros::TransformBroadcaster brOpGPS;
    geometry_msgs::TransformStamped transformStampedG;
    transformStampedG.header.stamp = ros::Time(t);
    transformStampedG.header.frame_id = "worldGPS";    //reference frame
    transformStampedG.child_frame_id = "world";
    transformStampedG.transform.translation.x = WGPS_T_WVIO_viz(0,3); //read & send the pos
    transformStampedG.transform.translation.y = WGPS_T_WVIO_viz(1,3);
    transformStampedG.transform.translation.z = WGPS_T_WVIO_viz(2,3);

    Eigen::Quaterniond q_upTemp;
    q_upTemp = Eigen::Quaterniond(WGPS_T_WVIO_viz.block<3, 3>(0, 0));
    transformStampedG.transform.rotation.x = q_upTemp.x();
    transformStampedG.transform.rotation.y = q_upTemp.y();
    transformStampedG.transform.rotation.z = q_upTemp.z();
    transformStampedG.transform.rotation.w = q_upTemp.w();

    //static_broadcaster.sendTransform(static_transformStamped);
    brOpGPS.sendTransform(transformStampedG);


    

    mPoseMap.unlock();
}

void GlobalOptimization::getGlobalOdom(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ)
{
    odomP = lastP;
    odomQ = lastQ;
}

void GlobalOptimization::inputGPS(double t, double latitude, double longitude, double altitude, double posAccuracy)
{
	double xyz[3];
	GPS2XYZ(latitude, longitude, altitude, xyz);
	vector<double> tmp{xyz[0], xyz[1], xyz[2], posAccuracy};
	GPSPositionMap[t] = tmp;
    newGPS = true;

    //3. Begin - GPS CSV wrighting
    std::ofstream foutF("GPS_bell412_dataset6_ppk.txt", std::ios::app);
    gpsPosCounter++;
    foutF.setf(std::ios::fixed, std::ios::floatfield);
    foutF.precision(0);
    foutF << gpsPosCounter << " ";
    foutF.precision(9);
    foutF << t  << " "
          << xyz[0]  << " "
          << xyz[1]  << " "
          << xyz[2]  << std::endl;
    //3. End - GPS CSV wrighting

}

void GlobalOptimization::optimize()
{
    while(true)
    {
        if(newGPS)
        {
            newGPS = false;
            printf("global optimization\n");
            TicToc globalOptimizationTime;

            ceres::Problem problem;
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            //options.minimizer_progress_to_stdout = true;
            //options.max_solver_time_in_seconds = SOLVER_TIME * 3;
            options.max_num_iterations = 5;
            ceres::Solver::Summary summary;
            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(1.0);
            ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();

            //add param
            mPoseMap.lock();
            int length = localPoseMap.size();
            // w^t_i   w^q_i
            double t_array[length][3];
            double q_array[length][4];
            map<double, vector<double>>::iterator iter;
            iter = globalPoseMap.begin();
            for (int i = 0; i < length; i++, iter++)
            {
                t_array[i][0] = iter->second[0];
                t_array[i][1] = iter->second[1];
                t_array[i][2] = iter->second[2];
                q_array[i][0] = iter->second[3];
                q_array[i][1] = iter->second[4];
                q_array[i][2] = iter->second[5];
                q_array[i][3] = iter->second[6];
                problem.AddParameterBlock(q_array[i], 4, local_parameterization);
                problem.AddParameterBlock(t_array[i], 3);
            }

            map<double, vector<double>>::iterator iterVIO, iterVIONext, iterGPS;
            int i = 0;
            int printOnceInTerminal = 1; //to print GPS map size
            //std::ofstream foutE("GPSPositionsLH.txt", std::ios::app);
            //int gpsPosCounter =0;

            for (iterVIO = localPoseMap.begin(); iterVIO != localPoseMap.end(); iterVIO++, i++)
            {
                //vio factor
                iterVIONext = iterVIO;
                iterVIONext++;
                if(iterVIONext != localPoseMap.end())
                {
                    Eigen::Matrix4d wTi = Eigen::Matrix4d::Identity();
                    Eigen::Matrix4d wTj = Eigen::Matrix4d::Identity();
                    wTi.block<3, 3>(0, 0) = Eigen::Quaterniond(iterVIO->second[3], iterVIO->second[4], 
                                                               iterVIO->second[5], iterVIO->second[6]).toRotationMatrix();
                    wTi.block<3, 1>(0, 3) = Eigen::Vector3d(iterVIO->second[0], iterVIO->second[1], iterVIO->second[2]);
                    wTj.block<3, 3>(0, 0) = Eigen::Quaterniond(iterVIONext->second[3], iterVIONext->second[4], 
                                                               iterVIONext->second[5], iterVIONext->second[6]).toRotationMatrix();
                    wTj.block<3, 1>(0, 3) = Eigen::Vector3d(iterVIONext->second[0], iterVIONext->second[1], iterVIONext->second[2]);
                    Eigen::Matrix4d iTj = wTi.inverse() * wTj;
                    Eigen::Quaterniond iQj;
                    iQj = iTj.block<3, 3>(0, 0);
                    Eigen::Vector3d iPj = iTj.block<3, 1>(0, 3);

                    ceres::CostFunction* vio_function = RelativeRTError::Create(iPj.x(), iPj.y(), iPj.z(),
                                                                                iQj.w(), iQj.x(), iQj.y(), iQj.z(),
                                                                                0.1, 0.01);
                    problem.AddResidualBlock(vio_function, NULL, q_array[i], t_array[i], q_array[i+1], t_array[i+1]);
                }
                //gps factor
                double t = iterVIO->first;
                iterGPS = GPSPositionMap.find(t);
                if (iterGPS != GPSPositionMap.end())
                {
                    ceres::CostFunction* gps_function = TError::Create(iterGPS->second[0], iterGPS->second[1], 
                                                                       iterGPS->second[2], iterGPS->second[3]);
                    //printf("inverse weight %f \n", iterGPS->second[3]);
                    problem.AddResidualBlock(gps_function, loss_function, t_array[i]);

                }

                //write the GPS locations to a file
                //read map
                /*map<double, vector<double>>::iterator iterGPS;
                iterGPS = GPSPositionMap.begin();
                if(printOnceInTerminal)
                {   
                    optcounter++;
                    printOnceInTerminal = 0;
                    std::cout << "GPS Map size: ****************** " << GPSPositionMap.size() << "opticounter" << optcounter << std::endl;
                    if(optcounter==327) //327
                    {
                        for (iterGPS = GPSPositionMap.begin(); iterGPS != GPSPositionMap.end(); iterGPS++)
                        {   //add file writing fuction
                            std::cout << "GPS Pos :" << iterGPS->second[0] << iterGPS->second[1] << iterGPS->second[2] << "***************" << std::endl;
                            gpsPosCounter++;
                            foutE.setf(std::ios::fixed, std::ios::floatfield);
                            foutE.precision(0);
                            foutE << gpsPosCounter << " ";
                            foutE.precision(6);
                            foutE << iterGPS->second[0]  << " "
                                << iterGPS->second[1]  << " "
                                << iterGPS->second[2]  << std::endl;
                        }
                    }
                    
                    std::cout << "Number of Optimizations: ****************** " << optcounter << std::endl;
                }*/
                //save to file


            }

            //foutE.close();
            //mPoseMap.unlock();
            ceres::Solve(options, &problem, &summary);
            //std::cout << summary.BriefReport() << "\n";

            //rav begin
            //add the static transformation here - WGPS_T_WVIO - rav
            Eigen::Matrix4d initRotAfterOpti;
            initRotAfterOpti <<  0.495238,   0.865899,    0.0704165,  5.9154,
                                 0.867781,  -0.496895,    0.00713753, 2.29296,
                                 0.04117,    0.0575713,  -0.997492,   2.68105,
                                 0,          0,           0,          1;
            
            Eigen::Quaterniond initglobalQ;
            initglobalQ = initRotAfterOpti.block<3, 3>(0, 0);
            Eigen::Vector3d initglobalP = initRotAfterOpti.block<3, 1>(0, 3);

            // update global pose
            //mPoseMap.lock();
            iter = globalPoseMap.begin();
            for (int i = 0; i < length; i++, iter++)
            {
            	vector<double> globalPose{t_array[i][0], t_array[i][1], t_array[i][2],
            							  q_array[i][0], q_array[i][1], q_array[i][2], q_array[i][3]};
            	iter->second = globalPose;
            	if(i == length - 1)
            	{
            	    Eigen::Matrix4d WVIO_T_body = Eigen::Matrix4d::Identity(); 
            	    Eigen::Matrix4d WGPS_T_body = Eigen::Matrix4d::Identity();
            	    double t = iter->first;
            	    WVIO_T_body.block<3, 3>(0, 0) = Eigen::Quaterniond(localPoseMap[t][3], localPoseMap[t][4], 
            	                                                       localPoseMap[t][5], localPoseMap[t][6]).toRotationMatrix();
            	    WVIO_T_body.block<3, 1>(0, 3) = Eigen::Vector3d(localPoseMap[t][0], localPoseMap[t][1], localPoseMap[t][2]);
            	    WGPS_T_body.block<3, 3>(0, 0) = Eigen::Quaterniond(globalPose[3], globalPose[4], 
            	                                                        globalPose[5], globalPose[6]).toRotationMatrix();
            	    WGPS_T_body.block<3, 1>(0, 3) = Eigen::Vector3d(globalPose[0], globalPose[1], globalPose[2]);
            	    WGPS_T_WVIO = WGPS_T_body * WVIO_T_body.inverse();
                    //rav
                    //WGPS_T_WVIO = WGPS_T_WVIO*initRotAfterOpti.transpose();
            	}
            }
            updateGlobalPath();
            //printf("global time %f \n", globalOptimizationTime.toc());
            mPoseMap.unlock();
        }
        std::chrono::milliseconds dura(2000);
        std::this_thread::sleep_for(dura);
    }
	return;
}


void GlobalOptimization::updateGlobalPath()
{
    global_path.poses.clear();
    map<double, vector<double>>::iterator iter;
    for (iter = globalPoseMap.begin(); iter != globalPoseMap.end(); iter++)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time(iter->first);
        pose_stamped.header.frame_id = "worldGPS";
        pose_stamped.pose.position.x = iter->second[0];
        pose_stamped.pose.position.y = iter->second[1];
        pose_stamped.pose.position.z = iter->second[2];
        pose_stamped.pose.orientation.w = iter->second[3];
        pose_stamped.pose.orientation.x = iter->second[4];
        pose_stamped.pose.orientation.y = iter->second[5];
        pose_stamped.pose.orientation.z = iter->second[6];
        global_path.poses.push_back(pose_stamped);

        //save the pose
        //convert to rotation matrix 
        //write to file
    }

    //4. add variable increment
    updateGlobalPathCount++;

    //save results for KITTI evaluation tool
    int length = globalPoseMap.size();
    //std::cout << "globalPoseMap size " << length << '\n';
    iter = globalPoseMap.begin();
    //std::cout << "globalPoseMap first location:" << " x:" << iter-> second[0] << " y:" << iter-> second[1] << " z:" << iter-> second[2] << '\n';
    //calculate the rotation matrix
    //Eigen::Quaterniond globalQ_forInit;
    //Eigen::Matrix3d globalR_forInit = globalQ_forInit.normalized().toRotationMatrix(); 
    //std::cout << "globalPoseMap first location:" << " x:" << iter-> second[0] << " y:" << iter-> second[1] << " z:" << iter-> second[2] << '\n';

    Eigen::Quaterniond odomQ;
    Eigen::Vector3d odomP;
    Eigen::Quaterniond gtQ;
    Eigen::Vector3d gtP;
    map<double, vector<double>>::iterator iter2;
    iter = localPoseMap.begin();
    iter2 = globalPoseMap.begin();
    //Read out the local and global pose from here
    //convert and find the transformation
    Eigen::Quaterniond odomQfirst;
    Eigen::Vector3d odomPfirst;
    Eigen::Quaterniond gtQfirst;
    Eigen::Vector3d gtPfirst;
    //to save full map
    Eigen::Quaterniond odomQAll;
    Eigen::Vector3d odomPAll;

    //local pose
    map<double, vector<double>>::iterator iter_lOdom;
    iter_lOdom = localPoseMap.begin();
    //global pose
    map<double, vector<double>>::iterator iter_gOdom;
    iter_gOdom = globalPoseMap.begin();
    //odom transformation
    //add it to the first update - where - up top
    //print once
    int printOnceInTerminal = 1;
    //write the whole map to a text file
    map<double, vector<double>>::iterator iterFull_gOdom;
    iterFull_gOdom = globalPoseMap.begin();
    //full frame count iter

    // time sequence check-k  
    //double time_first = iter->first;
    for(int j = 0;j < GTframeCount; j++, iter++, iter2++){ // go to the current frame
    }
    std::ofstream foutC("resultsOdom.txt", std::ios::app);  
    std::ofstream foutD("resultsGt.txt", std::ios::app); 
    //init text file for full path after optimization
    //std::ofstream foutE("resultsGtFullAfterOpti.txt", std::ios::app);

    for (int i = GTframeCount; i < length; i++, iter++, iter2++)
    {
                
		GTframeCount++;                

                
                odomP.x() = iter->second[0];
                odomP.y() = iter->second[1];
                odomP.z() = iter->second[2];
                odomQ.w() = iter->second[3];
                odomQ.x() = iter->second[4];
                odomQ.y() = iter->second[5];
                odomQ.z() = iter->second[6];

                //read the begining of map always - rav
                odomPfirst.x() = iter_lOdom->second[0];
                odomPfirst.y() = iter_lOdom->second[1];
                odomPfirst.z() = iter_lOdom->second[2];
                odomQfirst.w() = iter_lOdom->second[3];
                odomQfirst.x() = iter_lOdom->second[4];
                odomQfirst.y() = iter_lOdom->second[5];
                odomQfirst.z() = iter_lOdom->second[6];


                
                //time sequence check-k 
		        //std::cout <<  iter->first - time_first << "," << odomP.x() <<  "|" ;  // ok correct time squence saved
   
		    Eigen::Quaterniond globalQ;
    		globalQ = WGPS_T_WVIO_viz.block<3, 3>(0, 0) * odomQ;
    		Eigen::Vector3d globalP = WGPS_T_WVIO_viz.block<3, 3>(0, 0) * odomP + WGPS_T_WVIO_viz.block<3, 1>(0, 3);

            //calculate first pose 
            Eigen::Quaterniond globalQfirst;
            globalQfirst = WGPS_T_WVIO_viz.block<3, 3>(0, 0) * odomQfirst;
            Eigen::Vector3d globalPfirst = WGPS_T_WVIO_viz.block<3, 3>(0, 0) * odomPfirst + WGPS_T_WVIO_viz.block<3, 1>(0, 3);
            
   		
     		if(GTframeCount>0)
    		{
              //calculate first pose rotation - rav
                if (printOnceInTerminal)
                {   
                    //printOnceInTerminal = 0;
                    std::cout << "--------local PoseMap pose begin --------------" << '\n';
                    std::cout << "localPoseMap first location:" << " x:" << globalPfirst[0] << " y:" << globalPfirst[1] << " z:" << globalPfirst[2] << '\n';
                    Eigen::Matrix3d globalRfirst = globalQfirst.normalized().toRotationMatrix();
                    //std::cout << "localPoseMap first rotation:" << '\n' << globalRfirst << '\n';
                }
             
		      Eigen::Matrix3d globalR = globalQ.normalized().toRotationMatrix();   
              //std::cout << "Gloabal Rotm: " << globalR << std::endl;
	    	  foutC.setf(std::ios::fixed, std::ios::floatfield);
	    	  foutC.precision(0);
		      //foutC << header.stamp.toSec() * 1e9 << ",";
              foutC << GTframeCount << " "; //odom
		      foutC.precision(6);
		      foutC << globalR(0,0) << " "
				    << globalR(0,1) << " "
				    << globalR(0,2) << " "
				    << globalP.x()  << " "
				    << globalR(1,0) << " "
				    << globalR(1,1) << " "
				    << globalR(1,2) << " "
				    << globalP.y()  << " "
				    << globalR(2,0) << " "
				    << globalR(2,1) << " "
				    << globalR(2,2) << " "
				    << globalP.z()  << std::endl;

                //write the whole map to a new file
                if(printOnceInTerminal){
                    std::cout << "global Map size: " << globalPoseMap.size() << '\n';
                    std::cout << "Map Update Counter:  " << updateGlobalPathCount << '\n';
                }


                //4. Begin - Fusion CSV writing
                if(updateGlobalPathCount >= 280) //bell412_dataset1 - 149|bell412_dataset5 - 136 fusion| dataset3 - 150 | dataset4 - 155
                //if(0) //quarry1-102 | quarry2 - 132 (start-450 stop-269) | quarry3-240
                {
                    int GTframeCountFull = 0;
                    //std::ofstream foutG("Fusion_LH.txt", std::ios::app);
                    std::ofstream foutG("Fusion_bell412_dataset6_ppk.txt", std::ios::app);
                    for(iterFull_gOdom = globalPoseMap.begin(); iterFull_gOdom != globalPoseMap.end(); iterFull_gOdom++)
                    {
                        //read map
                        odomPAll.x() = iterFull_gOdom->second[0];
                        odomPAll.y() = iterFull_gOdom->second[1];
                        odomPAll.z() = iterFull_gOdom->second[2];
                        odomQAll.w() = iterFull_gOdom->second[3];
                        odomQAll.x() = iterFull_gOdom->second[4];
                        odomQAll.y() = iterFull_gOdom->second[5];
                        odomQAll.z() = iterFull_gOdom->second[6];

                        //calculate pose
                        Eigen::Quaterniond globalQAll;
                        globalQAll = WGPS_T_WVIO_viz.block<3, 3>(0, 0) * odomQAll;
                        Eigen::Vector3d globalPAll = WGPS_T_WVIO_viz.block<3, 3>(0, 0) * odomPAll + WGPS_T_WVIO_viz.block<3, 1>(0, 3);
                        Eigen::Matrix3d globalRAll = globalQAll.normalized().toRotationMatrix();  
                        //std::cout << "Gloabal Rotm: " << globalRAll << std::endl;

                        GTframeCountFull++;
                        foutG.setf(std::ios::fixed, std::ios::floatfield);
                        foutG.precision(0);
                        foutG << GTframeCountFull << " ";
                        foutG.precision(9);
                        foutG << ros::Time(iterFull_gOdom->first) << " " //added time - check
                            << globalRAll(0,0) << " "
                            << globalRAll(0,1) << " "
                            << globalRAll(0,2) << " "
                            << globalPAll.x()  << " "
                            << globalRAll(1,0) << " "
                            << globalRAll(1,1) << " "
                            << globalRAll(1,2) << " "
                            << globalPAll.y()  << " "
                            << globalRAll(2,0) << " "
                            << globalRAll(2,1) << " "
                            << globalRAll(2,2) << " "
                            << globalPAll.z()  << std::endl;
                    }
                }
                //4. End - Fusion CSV writing
                

    		}

		        gtP.x() = iter2->second[0];
                gtP.y() = iter2->second[1];
                gtP.z() = iter2->second[2];
                gtQ.w() = iter2->second[3];
                gtQ.x() = iter2->second[4];
                gtQ.y() = iter2->second[5];
                gtQ.z() = iter2->second[6];

                //global map first location read - rav
                gtPfirst.x() = iter_gOdom->second[0];
                gtPfirst.y() = iter_gOdom->second[1];
                gtPfirst.z() = iter_gOdom->second[2];
                gtQfirst.w() = iter_gOdom->second[3];
                gtQfirst.x() = iter_gOdom->second[4];
                gtQfirst.y() = iter_gOdom->second[5];
                gtQfirst.z() = iter_gOdom->second[6];
    	
   		
     		if(GTframeCount>0)
    		{ 
              //calculate global first pose -
                if (printOnceInTerminal)
                {   
                    printOnceInTerminal = 0;
                    std::cout << "globalPoseMap first location:" << " x:" << gtPfirst[0] << " y:" << gtPfirst[1] << " z:" << gtPfirst[2] << '\n';
                    Eigen::Matrix3d gtRfirst = gtQfirst.normalized().toRotationMatrix();
                    //std::cout << "globalPoseMap first rotation:" << '\n' << gtRfirst << '\n';
                    std::cout << "--------globalPoseMap pose end --------------" << '\n';
                } 
                
		      Eigen::Matrix3d gtR = gtQ.normalized().toRotationMatrix();   
	    	  foutD.setf(std::ios::fixed, std::ios::floatfield);
	    	  foutD.precision(0);
		      //foutC << header.stamp.toSec() * 1e9 << ",";
              foutD << GTframeCount << " "; //gt
		      foutD.precision(6);
		      foutD << gtR(0,0) << " "
				    << gtR(0,1) << " "
				    << gtR(0,2) << " "
				    << gtP.x()  << " "
				    << gtR(1,0) << " "
				    << gtR(1,1) << " "
				    << gtR(1,2) << " "
				    << gtP.y()  << " "
				    << gtR(2,0) << " "
				    << gtR(2,1) << " "
				    << gtR(2,2) << " "
				    << gtP.z()  << std::endl;
    		}
    }
     // time sequence check -k
    //std::cout <<  std::endl;
    //std::cout <<  localPoseMap.end()->first <<std::endl;
    foutC.close();
    foutD.close();
    //foutE.close();
    //save the last pose when everything finish running -> 
    //fix the first orientation from the last saved pose-> transformation of GPS to WORLD
    //re-run the optimization
    //Run the evaluation again
}


