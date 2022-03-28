#pragma once
extern "C" {
#include "lidardemoData.h"
#include "lidardemoAPI.h"
}
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/serialization/string.hpp>
#include "../../src/LidarBase.h"

void status_check(int status, std::string error_msg);


#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define BOLDWHITE   "\033[1m\033[37m"
#define BOLDBLUE    "\033[1m\033[34m"

class SenseLidar : public LidarBase {
   private:
    int lidar_id_;
    int status_;
    std::string lidar_ip_;
    int stream_id_;
    long int internal_frame_count_;
    SP_LIDAR_3D_DATA data_3d_;
    SP_LIDAR_2D_DATA data_2d_;

   public:
    SenseLidar(std::string lidar_ip);
    ~SenseLidar();
    pcl::PointCloud<pcl::PointXYZI> get_pcd() override;
    pcl::PointCloud<pcl::PointXYZI> get_pcl();
    pcl::PointCloud<pcl::PointXYZI> get_xyzi_frame();
 //   void get_intensity_pcl(pcl::PointCloud<pcl::PointXYZI>& , cv::Mat& );

    void start_3d_stream();
    void stop_stream();
    void terminate_lidar();
};

