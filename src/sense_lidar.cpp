#include "sense_ros_lib/sense_lidar.h"
#include "pcl/common/io.h"
int cameraId = 1;
#define MAX_TRESHOLD 15


void status_check(int status_, std::string error_msg) {
    if (status_ != 0) {
        std::cerr << error_msg << "  " << status_ << '\n';
        exit(1);
    }
}

SenseLidar::SenseLidar(std::string lidar_ip) : LidarBase(lidar_ip)
{
    status_ = SP_initLidar((char*)lidar_ip.c_str(), &lidar_id_);
    status_check(status_, "LiDAR initialization failed");
    if (status_ != 0) return;
    std::cout << GREEN << "Sense Lidar Initialized..." << RESET << std::endl;
    //status_ = SP_startStream(lidar_id_, 0, 1, 0, 0, 0, &stream_id_);  // requests 3d data
    //status_check(status_, "Stream start failed");
    //if (status_ != 0) return;
    std::cout << GREEN << "LiDAR Stream Initialized..." << RESET << std::endl;
    lidar_ip_ = lidar_ip;
    internal_frame_count_ = 0;
}



pcl::PointCloud<pcl::PointXYZI> SenseLidar::get_pcl() {
    status_ = SP_get3dData(stream_id_, &data_3d_);
    int status2d_ = SP_get2dData(stream_id_, &data_2d_);
    status_check(status_, "Read failure on 3d frame");
    status_check(status2d_, "Read failure on 2d frame");
    pcl::PointCloud<pcl::PointXYZI> new_frame;
    //new_frame.header.stamp = data_3d_.timeStamp;
    new_frame.header.seq = data_3d_.frameNum;
    std::size_t num_points = sizeof(data_3d_.points) / sizeof(SP_LIDAR_POINT);
    for (std::size_t point_index = 0; point_index < num_points; ++point_index) {
          pcl::PointXYZI temp_point;

            temp_point.x = data_3d_.points[point_index].x;
            temp_point.y = data_3d_.points[point_index].y;
            temp_point.z = data_3d_.points[point_index].z;

            temp_point.intensity = data_2d_.intensity[point_index];
            //temp_point.intensity = 0.0;
        new_frame.push_back(temp_point);
    }
    return new_frame;
}


void SenseLidar::start_3d_stream() {
    status_ = SP_startStream(lidar_id_, 0, 1, 0, 0, 0, &stream_id_);  // requests 3d data
    status_check(status_, "Stream start failed");
    internal_frame_count_ = 0;
}

void SenseLidar::stop_stream() {
    status_ = SP_stopStream(stream_id_);
    status_check(status_, "Stop stream failed");
}

void SenseLidar::terminate_lidar() {
    status_ = SP_terminateLidar(lidar_id_);
    status_check(status_, "LiDAR terminate failed");
}


SenseLidar::~SenseLidar(){
  stop_stream();
  std::cout << RED << "Stopped streaming" << std::endl;
  terminate_lidar();
  std::cout << "Terminated LiDAR" << RESET << std::endl;
}

pcl::PointCloud<pcl::PointXYZI> SenseLidar::get_pcd()
{
    /*
    pcl::PointCloud<pcl::PointXYZI> temp = get_pcl();
    pcl::PointCloud<pcl::PointXYZI> out_cloud;
    pcl::copyPointCloud(temp, out_cloud);
    */

    pcl::PointCloud<pcl::PointXYZI> t;

    ///Må starte og stoppe streamen hver gang man henter en frame
    ///Tror den har en intern buffer som overflower om man holder streamen oppe mellom hver gang
    status_ = SP_startStream(lidar_id_, 1, 1, 0, 0, 0, &stream_id_);  // requests 3d data
    status_check(status_, "Stream start failed");
    if(status_ == 0)
    {
        t = get_pcl();
    }
    else{
        std::cout << "Error in stream start. Try rebooting LIDAR\n";
        t.resize(1000);
    }

    status_ = SP_stopStream(stream_id_);

    pcl::PointCloud<pcl::PointXYZI> pointCloudFix;
    for(auto &p : t.points)
    {

        /*
        if(p.z <= 15 && p.x <= 65.53 && p.y <= 65.53)
        {
            pointCloudFix.push_back(p);
        }
         */

        if(p.z <= MAX_TRESHOLD && p.x <= MAX_TRESHOLD && p.y <= MAX_TRESHOLD)
        {
            pointCloudFix.push_back(p);
        }

    }


    return pointCloudFix;
}
