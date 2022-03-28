#include <iostream>
#include <sense_ros_lib/sense_lidar.h>
#include <gp_ros/GPLidar.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
// Linux headers
#include <boost/asio.hpp>
#include <chrono>
#include <ncurses.h>
#include <thread>
#include <mutex>

struct imu_data {
    Eigen::Vector3d linear_acc;
    Eigen::Vector3d angular_v;
    Eigen::Quaternionf orientation;
};

void print_data(Eigen::Quaternionf &ori, Eigen::Vector3d &angular_v, Eigen::Vector3d &linear_acc);
imu_data get_orientation(boost::asio::serial_port &port);
void create_fragment(uint16_t *fc, std::shared_ptr<LidarBase> &lidar, imu_data &im_d, pcl::PointCloud<pcl::PointXYZI> &c_cloud, const std::string &out_path);
void create_odom(uint16_t *oc, std::shared_ptr<LidarBase> &lidar, imu_data &im_d, const std::string &out_path);
void thread_imu_get(boost::asio::serial_port &port, imu_data &imu_d);
void fetch_current_cloud(std::shared_ptr<LidarBase> &lidar, pcl::PointCloud<pcl::PointXYZI> &c_cloud);

bool received_message = false;
uint8_t last_received_message_number;
uint16_t file_counter = 0;
uint16_t odom_counter = 0;
static volatile bool p_run = true;

std::mutex imu_mutex;
std::mutex cloud_mutex;

///Abstraksjonsklasse for LIDARtype
LidarBase* LidarFactory(lidar_type lt, std::string &lip)
{
    switch(lt)
    {
        case lidar_type::SENSE_LIDAR:
            return new SenseLidar(lip);
        case lidar_type::GP_LIDAR:
            return new GPLidar(lip);
        default:
            return nullptr;
    }
}

int main() {
    Eigen::Quaternionf zero_orientation;
    Eigen::Quaternionf orientation;
    std::cout << "Welcome to cool lidar scanner " << "\U0001F60E" << std::endl;

    const std::string OUTPUT_DIR = "../data";

    //TODO ta inn lidartype som parameter
    std::string lidar_ip_sense = "192.168.1.20";
    std::string lidar_ip_gp = "192.168.10.226";

    std::shared_ptr<LidarBase> lidar(LidarFactory(lidar_type::SENSE_LIDAR, lidar_ip_sense));

    ///Setter opp serial port
    boost::asio::io_service io;
    boost::asio::serial_port port(io);
    try
    {
        port.open("/dev/ttyACM0");
    } catch (boost::exception &e) {
        std::cerr << "Error opening serial port" << std::endl;
        return -1;
    }

    port.set_option(boost::asio::serial_port_base::baud_rate(115200));


    ///Lager tråd som kontinuerlig leser og oppdaterer imu-data
    imu_data imu_d;
    pcl::PointCloud<pcl::PointXYZI> current_cloud;
    std::thread imu_reader_t(thread_imu_get, std::ref(port), std::ref(imu_d));
    //std::thread cloud_reader_t(fetch_current_cloud, std::ref(lidar), std::ref(current_cloud));
    imu_reader_t.detach();
    //cloud_reader_t.detach();


    while(true) {

        std::cout << R"(Enter 'f' to enter fragment mode, 'o' to enter odometry mode, or 'q' to quit)" << std::endl;
        char input;
        std::cin >> input;

        if (input == 'f')
        {
            create_fragment(&file_counter, lidar, imu_d, current_cloud, OUTPUT_DIR);
        }
        else if (input == 'o')
        {
            create_odom(&odom_counter, lidar, imu_d, OUTPUT_DIR);
        }
        else if (input == 'p')
        {
            //std::lock_guard<std::mutex> guard(imu_mutex);
            auto euler = imu_d.orientation.toRotationMatrix().eulerAngles(0, 1, 2);
            double roll = euler[0] * (180/M_PI);
            double pitch = euler[1] * (180/M_PI);
            double yaw = euler[2] * (180/M_PI);

            std::cout << "Roll: "<< roll << "\n"
                      << "Pitch: " << pitch << "\n"
                      << "Yaw:" << yaw << "\n";
        }
        else if (input == 'q')
        {
            p_run = false;
            ///Gir tråd tid til å bli ferdig
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            return 0;
        }
    }
}

void print_data(Eigen::Quaternionf &ori, Eigen::Vector3d &angular_v, Eigen::Vector3d &linear_acc)
{
    std::cout << "Orientation Quaternion:" << "\n"
    << "W: " << ori.w() << "\n"
    << "X: " << ori.x() << "\n"
    << "Y: " << ori.y() << "\n"
    << "Z: " << ori.z() << std::endl;

    std::cout << "Linear acceleration:" << '\n'
              << "X: " << linear_acc.x() << '\n'
              << "Y: " << linear_acc.y() << '\n'
              << "Z: " << linear_acc.z() <<'\n';

    std::cout << "Angular velocity:" << '\n'
              << "X: " << angular_v.x() << '\n'
              << "Y: " << angular_v.y() << '\n'
              << "Z: " << angular_v.z() << std::endl;


}


imu_data get_orientation(boost::asio::serial_port &port)
{

    const std::string delimiter = "\r\n";
    imu_data im;
    im.orientation = Eigen::Quaternionf(0,0,0,0);
    im.linear_acc = Eigen::Vector3d(0,0,0);
    im.angular_v = Eigen::Vector3d(0,0,0);
    int loops = 0;
    std::string stringput;
    while(loops < 1000)
    {
        int data_packet_start;

        boost::asio::streambuf input;
        char c;
        boost::asio::read(port,  boost::asio::buffer(&c, 1));
        stringput += c;
        while (stringput.length() >= 28) {
            //parse for data packets
            data_packet_start = stringput.find("$\x03");
            if (data_packet_start != std::string::npos) {
                if ((stringput.length() >= data_packet_start + 28) &&
                    (stringput.compare(data_packet_start + 26, 2, "\r\n") ==
                     0))  //check if positions 26,27 exist, then test values
                {
                    // get quaternion values
                    int16_t w = (((0xff & (char) stringput[data_packet_start + 2]) << 8) |
                                 (0xff & (char) stringput[data_packet_start + 3]));
                    int16_t x = (((0xff & (char) stringput[data_packet_start + 4]) << 8) |
                                 (0xff & (char) stringput[data_packet_start + 5]));
                    int16_t y = (((0xff & (char) stringput[data_packet_start + 6]) << 8) |
                                 (0xff & (char) stringput[data_packet_start + 7]));
                    int16_t z = (((0xff & (char) stringput[data_packet_start + 8]) << 8) |
                                 (0xff & (char) stringput[data_packet_start + 9]));

                    double wf = w / 16384.0;
                    double xf = x / 16384.0;
                    double yf = y / 16384.0;
                    double zf = z / 16384.0;

                    im.orientation = Eigen::Quaternionf(wf, xf, yf, zf);


                    // get gyro values
                    int16_t gx = (((0xff & (char) stringput[data_packet_start + 10]) << 8) |
                                  (0xff & (char) stringput[data_packet_start + 11]));
                    int16_t gy = (((0xff & (char) stringput[data_packet_start + 12]) << 8) |
                                  (0xff & (char) stringput[data_packet_start + 13]));
                    int16_t gz = (((0xff & (char) stringput[data_packet_start + 14]) << 8) |
                                  (0xff & (char) stringput[data_packet_start + 15]));
                    // calculate rotational velocities in rad/s
                    // without the last factor the velocities were too small
                    // http://www.i2cdevlib.com/forums/topic/106-get-angular-velocity-from-mpu-6050/
                    // FIFO frequency 100 Hz -> factor 10 ?
                    // seems 25 is the right factor
                    //TODO: check / test if rotational velocities are correct
                    double gxf = gx * (4000.0 / 65536.0) * (M_PI / 180.0) * 25.0;
                    double gyf = gy * (4000.0 / 65536.0) * (M_PI / 180.0) * 25.0;
                    double gzf = gz * (4000.0 / 65536.0) * (M_PI / 180.0) * 25.0;

                    im.angular_v = Eigen::Vector3d(gxf, gyf, gzf);
                    // get acelerometer values
                    int16_t ax = (((0xff & (char) stringput[data_packet_start + 16]) << 8) |
                                  0xff & (char) stringput[data_packet_start + 17]);
                    int16_t ay = (((0xff & (char) stringput[data_packet_start + 18]) << 8) |
                                  0xff & (char) stringput[data_packet_start + 19]);
                    int16_t az = (((0xff & (char) stringput[data_packet_start + 20]) << 8) |
                                  (0xff & (char) stringput[data_packet_start + 21]));
                    // calculate accelerations in m/s²
                    double axf = ax * (8.0 / 65536.0) * -9.81;
                    double ayf = ay * (8.0 / 65536.0) * -9.81;
                    double azf = az * (8.0 / 65536.0) * -9.81;
                    im.linear_acc = Eigen::Vector3d(axf, ayf, azf);


                    uint8_t received_message_number = stringput[data_packet_start + 25];

                    if (received_message) // can only check for continuous numbers if already received at least one packet
                    {
                        uint8_t message_distance = received_message_number - last_received_message_number;
                        if ( message_distance > 1 )
                        {
                            std::cerr << "Missed " << message_distance - 1 << " MPU6050 data packets from arduino." << std::endl;
                        }
                    }
                    else
                    {
                        received_message = true;
                    }
                    last_received_message_number = received_message_number;

                    stringput.erase(0, data_packet_start + 28); // delete everything up to and including the processed packet

                    return im;
                }
                else
                {
                    if (stringput.length() >= data_packet_start + 28)
                    {
                        stringput.erase(0, data_packet_start + 1); // delete up to false data_packet_start character so it is not found again
                    }
                    else
                    {
                        // do not delete start character, maybe complete package has not arrived yet
                        stringput.erase(0, data_packet_start);
                    }
                }
            } else {
                // no start character found in input, so delete everything
                stringput.clear();
            }
        }
        loops++;
    }
    return im;

}

void create_fragment(uint16_t *fc, std::shared_ptr<LidarBase> &lidar, imu_data &im_d, pcl::PointCloud<pcl::PointXYZI> &c_cloud, const std::string &out_path)
{
    std::cout << R"(Entering fragment creation mode...)" << std::endl;

    int internal_frame = 0;
    std::string i_fc = std::to_string((*fc));

    std::fstream fout;
    std::string cloud_dir = out_path + "/fragments/fragment_" + i_fc + "/clouds/";
    std::string quat_dir = out_path + "/fragments/fragment_" + i_fc + "/quaternions/";
    if(!boost::filesystem::is_directory(out_path + "/fragments/fragment_" + i_fc))
    {
        boost::filesystem::create_directories(cloud_dir);
        boost::filesystem::create_directory(quat_dir);
    }
    std::string quat_fname = "quaternions.csv";

    pcl::PointCloud<pcl::PointXYZI> big_fragment;
    pcl::PCDWriter cloud_writer = pcl::PCDWriter();

    fout.open(quat_dir + quat_fname, std::ios::out | std::ios::trunc);

    if(!fout.is_open())
    {
        return;
    }

    fout << "q_w,q_x,q_y,q_z,t" << std::endl;
    for(;;)
    {
        std::cout << R"(Enter 'n' to register new image in fragment, or 'q' to quit fragment mode)" << std::endl;
        char input;
        std::cin >> input;

        if (input == 'n')
        {
            std::cout << "Getting cloud... " << std::flush;
            pcl::PointCloud<pcl::PointXYZI> cloud = lidar->get_pcd();
            //pcl::PointCloud<pcl::PointXYZI> cloud = c_cloud;
            Eigen::Quaternionf t = im_d.orientation;

            cloud.sensor_orientation_ = t;
            pcl::PointCloud<pcl::PointXYZI> t_cloud;
            auto rotationMatrix = t.toRotationMatrix();
            auto eulerAngles = t.toRotationMatrix().eulerAngles(0, 1, 2);

            Eigen::Matrix4f test_rotate_y;
            test_rotate_y << cos(eulerAngles[1]), 0, sin(eulerAngles[1]), 0,
                                          0, 1, 0, 0,
                                          -sin(eulerAngles[1]), 0, cos(eulerAngles[1]), 0,
                                          0, 0, 0, 1;

            Eigen::Matrix4f temp_transform;
            temp_transform(0,0) = rotationMatrix(0, 0);
            temp_transform(0,1) = rotationMatrix(0, 1);
            temp_transform(0,2) = rotationMatrix(0, 2);

            temp_transform(1,0) = rotationMatrix(1, 0);
            temp_transform(1,1) = rotationMatrix(1, 1);
            temp_transform(1,2) = rotationMatrix(1, 2);

            temp_transform(2,0) = rotationMatrix(2, 0);
            temp_transform(2,1) = rotationMatrix(2, 1);
            temp_transform(2,2) = rotationMatrix(2, 2);
            temp_transform.col(3).setZero();
            temp_transform.row(3).setZero();
            temp_transform(3, 3) = 1;
            pcl::transformPointCloud(cloud, t_cloud, test_rotate_y);
            big_fragment += t_cloud;

            std::string f_name = "fragment_num_" + i_fc + "_frame_num_" + std::to_string(internal_frame) + ".pcd";
            internal_frame++;
            cloud_writer.write(cloud_dir + f_name, cloud, false);
            std::cout << "Done!" << std::endl;
            std::chrono::milliseconds timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()
            );
                fout << std::to_string(t.w()) << "," << std::to_string(t.x()) << ","
                 << std::to_string(t.y()) << "," << std::to_string(t.z()) << ","
                 << std::to_string(timestamp.count()) << std::endl;


        }
        else if(input == 'q')
        {
            cloud_writer.write(cloud_dir + "big_fragment" + i_fc + ".pcd", big_fragment, true);
            std::cout << "Wrote " + std::to_string(internal_frame) + " clouds in fragment #" + i_fc << "\n";
            (*fc)++;
            fout.close();
            return;
        } else {
            std::cout << "Invalid input.\n";
        }
    }

}


void create_odom(uint16_t *oc, std::shared_ptr<LidarBase> &lidar, imu_data &im_d, const std::string &out_path)
{
    std::cout << R"(Entering odometry creation mode...)" << std::endl;
    std::cout << R"(Press any key to quit odometry)" << std::endl;

    pcl::PCDWriter cloud_writer = pcl::PCDWriter();
    int internal_odom = 0;
    std::string i_oc = std::to_string((*oc));

    std::fstream fout;
    std::string cloud_dir = out_path + "/odometry/odometry_" + i_oc + "/clouds/";
    std::string quat_dir = out_path + "/odometry/odometry_" + i_oc + "/quaternions/";

    if(!boost::filesystem::is_directory(out_path + "/odometry/odometry" + i_oc))
    {
        boost::filesystem::create_directories(cloud_dir);
        boost::filesystem::create_directory(quat_dir);
    }

    ///Åpner og setter opp outputfil for quaternions
    std::string fname_ori = "quaternions.csv";
    fout.open(quat_dir + fname_ori, std::ios::out | std::ios::trunc);
    fout << "q_w,q_x,q_y,q_z,t" << std::endl;

    if(!fout.is_open())
    {
        return;
    }

    //TODO fjerne bruk av curses
    int i = 0;
    auto s = initscr();
    noecho();
    cbreak();
    nodelay(s, true);
    while(!i)
    {
        pcl::PointCloud<pcl::PointXYZI> cloud = lidar->get_pcd();
        cloud.sensor_orientation_ = im_d.orientation;
        std::string f_name = "odom_num_" + i_oc + "_frag_num_" + std::to_string(internal_odom) + ".pcd";
        cloud_writer.write(cloud_dir + f_name, cloud, false);

        Eigen::Quaternionf ori = im_d.orientation;
        std::chrono::microseconds timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()
        );
        fout << std::to_string(ori.w()) << "," << std::to_string(ori.x()) << ","
             << std::to_string(ori.y()) << "," << std::to_string(ori.z()) << ","
             << std::to_string(timestamp.count()) << std::endl;

        internal_odom++;

        usleep(1);
        i=getch();
        if(i>0)
            i=1;
        else
            i=0;
        //printw("in_odom\n");
    }

    nocbreak();
    echo();
    endwin();
    std::cout << "Wrote " + std::to_string(internal_odom) + " clouds in odom #" + i_oc << "\n";
    (*oc)++;

}



void thread_imu_get(boost::asio::serial_port &port, imu_data &imu_d)
{
    imu_data temp;
    while(p_run)
    {
        ///Lock guard virker for treig, selv med -O3
        //std::lock_guard<std::mutex> guard(imu_mutex);
        imu_mutex.lock();
        temp = get_orientation(port);
        imu_d = temp;
        imu_mutex.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}


///Trådfunksjon for å kontinuerlig hente skyer, brukes ikke for øyeblikket
void fetch_current_cloud(std::shared_ptr<LidarBase> &lidar,  pcl::PointCloud<pcl::PointXYZI> &c_cloud)
{
    int num_clouds = 0;
    pcl::PointCloud<pcl::PointXYZI> temp;
    while(p_run)
    {

        if(num_clouds > 10)
        {
            std::cout << "Got 10 clouds\n";
            num_clouds = 0;
        }
        ///Lock guard virker for treig, selv med -O3
        //std::lock_guard<std::mutex> guard(imu_mutex);
        cloud_mutex.lock();
        c_cloud = lidar->get_pcd();
        cloud_mutex.unlock();
        num_clouds++;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

