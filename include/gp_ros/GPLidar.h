//
// Created by axel on 22.02.2021.
//
#pragma once
#include <iostream>
#include <string>
#include <cstring>
#include <chrono>
#include <thread>
#include <vector>
#include <cmath>
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "../../src/LidarBase.h"

extern "C" {
#include "GpSdk.h"
}



class GPLidar : public LidarBase {
private:
    void status_check(int status_, std::string error_msg);
    GpDeviceHandle g_handle;
    GpCameraLensParam g_cam_param;
    int lidar_id_{};
    int integrationTime = 200;
    int integrationTime_temp = 0;
    GpStreamType streamType = GP_STREAM_TYPE_DEPTH;

    int status_;
    std::string lidar_ip_;
    int stream_id_{};
    long int internal_frame_count_;
    GpFrame *frames{};




public:
     GPLidar(std::string &lidar_ip);
    ~GPLidar();
    pcl::PointCloud<pcl::PointXYZI> get_pcd() override
    {
        return get_pcl(getPointsFromFrame());
    };
    std::vector<unsigned short> getPointsFromFrame();
    pcl::PointCloud<pcl::PointXYZI> get_pcl(std::vector<unsigned short> points);

};