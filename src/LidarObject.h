//
// Created by axel on 18.04.2021.
//

#ifndef MANUAL_IMU_LIDAROBJECT_H
#define MANUAL_IMU_LIDAROBJECT_H

#include <string>

enum lidar_type
{
    SENSE_LIDAR = 1,
    GP_LIDAR = 2,
};
class LidarObject {

public:
    LidarObject(lidar_type lt, std::string &lidar_ip);
    ~LidarObject();

private:
    std::string lidar_ip;

};


#endif //MANUAL_IMU_LIDAROBJECT_H
