//
// Created by axel on 18.04.2021.
//

#ifndef MANUAL_IMU_LIDARBASE_H
#define MANUAL_IMU_LIDARBASE_H

#include <string>
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>

enum lidar_type
{
    SENSE_LIDAR = 1,
    GP_LIDAR = 2,
};

class LidarBase
        {
public:
    LidarBase(std::string& lip) {};
    virtual ~LidarBase() = default;
    virtual pcl::PointCloud<pcl::PointXYZI> get_pcd() = 0;


};


#endif //MANUAL_IMU_LIDARBASE_H
