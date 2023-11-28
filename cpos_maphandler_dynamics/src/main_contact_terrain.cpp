#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <ros/package.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <fstream>
#include <iostream>

#define PI 3.141592
#define Rad2Deg 57.295791433 // 180.0/3.141592
#define Deg2Rad 0.017453289  // 3.141592/180.0

using namespace std;
// ROS::
ros::Publisher pub_bathy;
ros::Publisher pub_track_contact;

// Cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bathy(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_track_contact(new pcl::PointCloud<pcl::PointXYZ>);

// searching
pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

// read track position file
ifstream openFile;

struct struct_point
{
    float x;
    float y;
    float z;
    float z_req;
};

struct struct_track_point
{
    struct_point point[8];
};

struct_track_point track[4];

void funcTimerRun(const ros::TimerEvent &ev);
void funcPub(const ros::TimerEvent &ev);

int main(int argc, char *argv[])
{
    ROS_INFO("Find Contact Points of Track and Terrain Node");
    /// 1. Init
    ros::init(argc, argv, "contact_track_and_terrain_node");
    ros::NodeHandle nh;

    /// 3. Publish & Subscriber
    pub_bathy = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("jeju_terrain/cloud", 10);
    pub_track_contact = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("vehicle/track_contact/cloud", 10);

    // 4. Load file
    /// 4-1. Map file
    std::string filePath_bathy = ros::package::getPath("cpos_maphandler_dynamics") + "/files/soilbasin_jeju_terrain_v3.ply";
    cloud_bathy->points.clear();
    pcl::PLYReader plyReader;
    if (plyReader.read(filePath_bathy, *cloud_bathy) == -1)
    {
        std::cout << "Couldn't read file soilbasin_jeju_terrain_v2.ply" << std::endl;
        ROS_INFO("Find Contact Points of Track and Terrain Node Finished.");
        return 0;
    }
    else
    {
        std::cout << "Loaded " << cloud_bathy->points.size()
                  << " data points from " << filePath_bathy
                  << std::endl;
        cloud_bathy->header.frame_id = "world";

        // set searching target
        kdtree.setInputCloud(cloud_bathy);
    }
    /// 4-2. track position file
    std::string filePath_track = ros::package::getPath("cpos_maphandler_dynamics") + "/files/track_node.txt";
    // read File
    openFile.open(filePath_track.data());
    cout << "Open File State : " << openFile.is_open() << endl;

    /// 5. Timer
    ros::Timer timer_run = nh.createTimer(ros::Duration(0.001), funcTimerRun, false);
    ros::Timer timer_pub = nh.createTimer(ros::Duration(2), funcPub, false);

    /// 6. spin
    ros::spin();

    openFile.close();

    ROS_INFO("Find Contact Points of Track and Terrain Node Finished.");

    return 0;
}

void funcPub(const ros::TimerEvent &ev)
{
    pcl_conversions::toPCL(ros::Time::now(), cloud_bathy->header.stamp);
    pub_bathy.publish(cloud_bathy);
}

void funcTimerRun(const ros::TimerEvent &ev)
{
    string line;
    if (getline(openFile, line))
    {
        const char *cline;
        cline = line.c_str();
        sscanf(cline, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,"
                      "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,"
                      "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,"
                      "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
               // get front left
               &track[0].point[0].x, &track[0].point[0].y, &track[0].point[0].z_req,
               &track[0].point[1].x, &track[0].point[1].y, &track[0].point[1].z_req,
               &track[0].point[2].x, &track[0].point[2].y, &track[0].point[2].z_req,
               &track[0].point[3].x, &track[0].point[3].y, &track[0].point[3].z_req,
               &track[0].point[4].x, &track[0].point[4].y, &track[0].point[4].z_req,
               &track[0].point[5].x, &track[0].point[5].y, &track[0].point[5].z_req,
               &track[0].point[6].x, &track[0].point[6].y, &track[0].point[6].z_req,
               &track[0].point[7].x, &track[0].point[7].y, &track[0].point[7].z_req,
               // get front right
               &track[1].point[0].x, &track[1].point[0].y, &track[1].point[0].z_req,
               &track[1].point[1].x, &track[1].point[1].y, &track[1].point[1].z_req,
               &track[1].point[2].x, &track[1].point[2].y, &track[1].point[2].z_req,
               &track[1].point[3].x, &track[1].point[3].y, &track[1].point[3].z_req,
               &track[1].point[4].x, &track[1].point[4].y, &track[1].point[4].z_req,
               &track[1].point[5].x, &track[1].point[5].y, &track[1].point[5].z_req,
               &track[1].point[6].x, &track[1].point[6].y, &track[1].point[6].z_req,
               &track[1].point[7].x, &track[1].point[7].y, &track[1].point[7].z_req,
               // get rear left
               &track[2].point[0].x, &track[2].point[0].y, &track[2].point[0].z_req,
               &track[2].point[1].x, &track[2].point[1].y, &track[2].point[1].z_req,
               &track[2].point[2].x, &track[2].point[2].y, &track[2].point[2].z_req,
               &track[2].point[3].x, &track[2].point[3].y, &track[2].point[3].z_req,
               &track[2].point[4].x, &track[2].point[4].y, &track[2].point[4].z_req,
               &track[2].point[5].x, &track[2].point[5].y, &track[2].point[5].z_req,
               &track[2].point[6].x, &track[2].point[6].y, &track[2].point[6].z_req,
               &track[2].point[7].x, &track[2].point[7].y, &track[2].point[7].z_req,
               // get rear right
               &track[3].point[0].x, &track[3].point[0].y, &track[3].point[0].z_req,
               &track[3].point[1].x, &track[3].point[1].y, &track[3].point[1].z_req,
               &track[3].point[2].x, &track[3].point[2].y, &track[3].point[2].z_req,
               &track[3].point[3].x, &track[3].point[3].y, &track[3].point[3].z_req,
               &track[3].point[4].x, &track[3].point[4].y, &track[3].point[4].z_req,
               &track[3].point[5].x, &track[3].point[5].y, &track[3].point[5].z_req,
               &track[3].point[6].x, &track[3].point[6].y, &track[3].point[6].z_req,
               &track[3].point[7].x, &track[3].point[7].y, &track[3].point[7].z_req);

        // cout << "point " << track[0].point[0].x << "\t" << track[0].point[0].y << "\t" << track[0].point[0].z_req << "\t";
        // cout << track[0].point[1].x << "\t" << track[0].point[1].y << "\t" << track[0].point[1].z_req << "\t";
        // cout << track[0].point[2].x << "\t" << track[0].point[2].y << "\t" << track[0].point[2].z_req << endl;

        cout << ros::Time::now() << " point " << track[0].point[0].x << "\t" << track[0].point[0].y << "\t" << track[0].point[0].z_req << endl;
    }
    else
    {
        cout << "file end" << endl;
        return;
    }

    cloud_track_contact->points.clear();
    cloud_track_contact->header.frame_id = "world";
    int numberOfneighbor = 4;
    float searching_radius = 0.5;
    for (int search_track_id = 0; search_track_id < 4; search_track_id++)
    {
        for (int search_track_point_id = 0; search_track_point_id < 8; search_track_point_id++)
        {
            pcl::PointXYZ searchPoint;
            searchPoint.x = track[search_track_id].point[search_track_point_id].x;
            searchPoint.y = track[search_track_id].point[search_track_point_id].y;
            track[search_track_id].point[search_track_point_id].z = 0;

            // search z
            for (float search_z_position = -1.0; search_z_position < 2.0; search_z_position = search_z_position + 0.1)
            {
                searchPoint.z = track[search_track_id].point[search_track_point_id].z_req + search_z_position;

                // Neighbors within radius search
                std::vector<int> pointIdxRadiusSearch;
                std::vector<float> pointRadiusSquaredDistance;
                if (kdtree.radiusSearch(searchPoint, searching_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
                {
                    if (pointIdxRadiusSearch.size() >= numberOfneighbor)
                    {
                        float sum_z = 0;
                        unsigned int numberOfPoint = pointIdxRadiusSearch.size();
                        for (unsigned int i = 0; i < numberOfPoint; i++)
                        {
                            sum_z = sum_z + (*cloud_bathy)[pointIdxRadiusSearch[i]].z;
                        }

                        // get z
                        track[search_track_id].point[search_track_point_id].z = sum_z / (float)numberOfPoint;

                        // cloud_2dmbes
                        pcl::PointXYZ p;
                        p.x = track[search_track_id].point[search_track_point_id].x;
                        p.y = track[search_track_id].point[search_track_point_id].y;
                        p.z = sum_z / (float)numberOfPoint;
                        cloud_track_contact->push_back(p);
                        break;
                    }
                }
            }
        }
    }
    pcl_conversions::toPCL(ros::Time::now(), cloud_track_contact->header.stamp);
    pub_track_contact.publish(cloud_track_contact);
}
