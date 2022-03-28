#include "gp_ros/GPLidar.h"


void GPLidar::status_check(int status_, std::string error_msg) {
    if (status_ != 0)
    {
        std::cerr << error_msg << "  " << status_ << '\n';
        exit(1);
    }
}

GPLidar::GPLidar(std::string &lidar_ip) : LidarBase(lidar_ip) {


    status_ = 0;
    gpSdkInit();
    int loops = 0;

    std::cout << lidar_ip <<std::endl;
    //std::cout << GP_INVALID_DEVICE_HANDLE <<std::endl;

    /* 创建设备句柄 */
    g_handle = gpCreateDeviceHandle(GP_DEVICE_TYPE_NET);
    std::cout << g_handle << std::endl;
    if (g_handle == (GpDeviceHandle) GP_INVALID_DEVICE_HANDLE) {
        status_ = -1;
        status_check(status_, "gpCreateDeviceHandle err\n");
    }

    GpDeviceParam param;
    int returnVal = 0;
    char ip[32] = { '\0' };

    snprintf(ip, sizeof(ip), "%s", "192.168.10.226");

    //strcpy(param.uri, lidar_ip.c_str());
    strcpy(param.uri, ip);
    std::cout << param.uri << std::endl;

    returnVal = gpOpenDevice(g_handle, &param);

    //Kall til SDK sendt
    if (returnVal != GP_EC_OK) {
        status_ = -1;
        status_check(status_, "Error opening device\n");
    }


    //Venter maks 5 sek på at LIDAR åpner
    while (gpGetDeviceState(g_handle) != GP_DEVICE_STATE_OPENED)
    {
        int g = gpGetDeviceState(g_handle);
        std::cout << g << std::endl;
        /*
        if (loops++ >= 5) {
            gpCloseDevice(g_handle);
            status_check(-1, "Failed to open device");
            break;
        }
         */
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

/*
    returnVal = gpStartStream(g_handle, streamType);
    //Kall til SDK sendt
    if (returnVal != GP_EC_OK) {
        status_ = -1;
        status_check(status_, "Error opening device\n");
    }

    loops = 0;
    //Venter på at stream starter
    while (gpGetStreamState(g_handle, streamType) != GP_STREAM_STATE_OPENED)
    {
        if (loops++ >= 300) {
            status_check(-1, "Stream start failed");
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "LiDAR Stream Initialized..." << std::endl;
    */
    //Trenger kameraparametere for å lage punktsky fra dybdeinfo
    //gpSetIntegrationTime(g_handle, integrationTime);
    //gpClearDeviceLastErrorCode(g_handle);
    gpGetCameraLensParam(g_handle, &g_cam_param);


}


GPLidar::~GPLidar(){

    int times = 0;

    gpStopStream(g_handle,GP_STREAM_TYPE_DEPTH);

    while(gpGetStreamState(g_handle,GP_STREAM_TYPE_DEPTH) != GP_STREAM_STATE_CLOSE)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "Stream stopped." << std::endl;

    gpCloseDevice(g_handle);

    while (gpGetDeviceState(g_handle) != GP_DEVICE_STATE_CLOSE)
    {
        if (times++ >= 3) {
            std::cout << "Device not properly closed." << std::endl;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "Device closed." << std::endl;
    gpReleaseDeviceHandle(g_handle);
    g_handle = (GpDeviceHandle) GP_INVALID_DEVICE_HANDLE;
    gpSdkUninit();

}

std::vector<unsigned short> GPLidar::getPointsFromFrame() {
    int returnVal;
    returnVal = gpStartStream(g_handle, streamType);
    //Kall til SDK sendt
    if (returnVal != GP_EC_OK) {
        status_ = -1;
        status_check(status_, "Error opening device\n");
    }

    int loops = 0;
    //Venter på at stream starter
    while (gpGetStreamState(g_handle, streamType) != GP_STREAM_STATE_OPENED)
    {
        if (loops++ >= 300) {
            status_check(-1, "Stream start failed");
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::vector<unsigned short> points;
    GpInt32_t timeout = 100;
    GpFrame *frame[1];
    GpInt32_t numframes;

    /* 阻塞读取图像数据流 返回值为读取到的总帧数 <= sizeof(frames) / sizeof(frames[0])  */
    //numframes gir lengden på framearray
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    numframes = gpReadStream(g_handle, frame, 1, timeout, streamType);
    int size = frame[0]->frameLen;
    //punktene er 2 byte, derfor unsigned short
    auto* pointData = (unsigned short*)frame[0]->data;


    for(int i = 0; i < size/2; i++)
    {
        //derefererer og legger til neste punkt
        points.push_back(*(pointData + i));
    }

    gpReleaseFrame(frame[0]);
    gpStopStream(g_handle,GP_STREAM_TYPE_DEPTH);
    while(gpGetStreamState(g_handle,GP_STREAM_TYPE_DEPTH) != GP_STREAM_STATE_CLOSE)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }


    return points;
}


pcl::PointCloud <pcl::PointXYZI> GPLidar::get_pcl(std::vector<unsigned short> points) {

    pcl::PointCloud<pcl::PointXYZI> pointCloud;


    int width = 640;
    int height = 160;
    float f_x = g_cam_param.fFocalLengthX;
    float f_y = g_cam_param.fFocalLengthY;
    float c_x = g_cam_param.fPrincipalPointX;
    float c_y = g_cam_param.fPrincipalPointY;

    pointCloud.height = height;
    pointCloud.width = width;
    pointCloud.is_dense = false;
    pointCloud.resize(height * width);

    float inv_f_x = 1/f_x;
    float inv_f_y = 1/f_y;
    int idx = 0;
    for( int r = 0; r < height; r++)
    {
        for (int c = 0; c < width; c++)
        {
            auto depth = points[idx];

            //std::cout << idx << std::endl;
            //pcl::PointXYZ temp;
            int u = c;
            int v = r;

            //TODO spesifiser format gjennom input

            //Normal
            /*
            pointCloud.points[idx].x = -((float)u - c_x) * (float)depth *(inv_f_x);
            pointCloud.points[idx].y = -((float)v - c_y) * (float)depth *(inv_f_y);
            pointCloud.points[idx].z = depth;
             */

            //In meters, no tf

/*
            pointCloud.points[idx].x = (float)depth/1000;
            pointCloud.points[idx].y = (-((float)u - c_x) * (float)depth *(inv_f_x))/1000;
            pointCloud.points[idx].z = (-((float)v - c_y) * (float)depth *(inv_f_y))/1000;
            */

            //Rotert 90 grader rundt X
            /*
            pointCloud.points[idx].x = (float)depth/1000;
            pointCloud.points[idx].y = -(-((float)v - c_y) * (float)depth *(inv_f_y))/1000;
            pointCloud.points[idx].z = (-((float)u - c_x) * (float)depth *(inv_f_x))/1000;
            */

            //Normal, meter
            /*
            pointCloud.points[idx].x = -((float)u - c_x) * (float)depth *(inv_f_x) / 1000;
            pointCloud.points[idx].y = -((float)v - c_y) * (float)depth *(inv_f_y) / 1000;
            pointCloud.points[idx].z = (float)depth / 1000;
            */


            pointCloud.points[idx].x = ((float)v - c_y) * (float)depth *(inv_f_y) / 1000;
            pointCloud.points[idx].y = -((float)u - c_x) * (float)depth *(inv_f_x) / 1000;
            pointCloud.points[idx].z = (float)depth / 1000;
            pointCloud.points[idx].intensity = 1.0;



            //Normal, meter, 90grader z
/*
            pointCloud.points[idx].x =  ((float)v - c_y) * (float)depth *(inv_f_y) / 1000;
            pointCloud.points[idx].y = -((float)u - c_x) * (float)depth *(inv_f_x) / 1000;
            pointCloud.points[idx].z = (float) depth / 1000;
*/

            //90 grader z, andre vei

            /*
            pointCloud.points[idx].x =  -((float)v - c_y) * (float)depth *(inv_f_y) / 1000;
            pointCloud.points[idx].y = ((float)u - c_x) * (float)depth *(inv_f_x) / 1000;
            pointCloud.points[idx].z = (float) depth / 1000;
            */






            //No tf
            /*
            pointCloud.points[idx].x = depth;
            pointCloud.points[idx].y =  -((float)u - c_x) * (float)depth *(inv_f_x);
            pointCloud.points[idx].z = -((float)v - c_y) * (float)depth *(inv_f_y);
            */
            idx++;
        }
    }


    pcl::PointCloud<pcl::PointXYZI> pointCloudFix;

    for(auto &p : pointCloud.points)
    {
        //Normal
    /*
    if(p.z != 65534)
    {
        pointCloudFix.push_back(p);
    }
     */
    //No_tf
    /*
     if(p.x != 65534)
     {
         pointCloudFix.push_back(p);
     }
      */
    //In_meters, no tf

     if(p.z <= 65.53 && p.x <= 65.53 && p.y <= 65.53)
     {
         pointCloudFix.push_back(p);
     }
    }


    return pointCloudFix;

}

