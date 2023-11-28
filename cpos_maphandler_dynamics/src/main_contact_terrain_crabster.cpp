#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <ros/package.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <std_msgs/Float64MultiArray.h>


#define PI 3.141592
#define Rad2Deg 57.295791433 // 180.0/3.141592
#define Deg2Rad 0.017453289  // 3.141592/180.0

using namespace std;
// ROS::
ros::Publisher pub_bathy;
ros::Publisher pub_foot_contact_cloud;
ros::Publisher pub_foot_contact;
ros::Subscriber sub_foot_contact_xyz;

// Cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bathy(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_foot_contact(new pcl::PointCloud<pcl::PointXYZ>);

// Contact XYZ
std_msgs::Float64MultiArray array_contact_z;

// searching
pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

struct struct_point
{
    float x;
    float y;
    float z;
    float z_req;
};
struct_point foot_point[6];

void funcPub(const ros::TimerEvent &ev);
void funcSub_foot_contact_xyz(const std_msgs::Float64MultiArray::ConstPtr &input);
void funcRealtimeRun(void);

int main(int argc, char *argv[])
{
    ROS_INFO("Find Contact Points of Foot and Terrain Node");
    /// 1. Init
    ros::init(argc, argv, "contact_foot_and_terrain_node");
    ros::NodeHandle nh;

    /// 3. Publish & Subscriber
    pub_bathy               = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>  ("jeju_terrain/cloud", 10);
    pub_foot_contact_cloud  = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>  ("crabster/foot/contact_cloud", 10);
    pub_foot_contact        = nh.advertise<std_msgs::Float64MultiArray>     ("crabster/foot/contact_z", 5);

    sub_foot_contact_xyz    = nh.subscribe<std_msgs::Float64MultiArray>  ("crabster/foot/contact_req", 18, &funcSub_foot_contact_xyz);

    // 4. Load file
    /// 4-1. Map file
    std::string filePath_bathy = ros::package::getPath("cpos_maphandler_dynamics") + "/files/soilbasin_jeju_terrain_v3.ply";
    // std::string filePath_bathy = ros::package::getPath("cpos_maphandler_dynamics") + "/files/jeju_bathymetry_soilbasin_frame_ascii.ply";
    cloud_bathy->points.clear();
    pcl::PLYReader plyReader;
    if (plyReader.read(filePath_bathy, *cloud_bathy) == -1)
    {
        std::cout << "Couldn't read file soilbasin_jeju_terrain_v5.ply" << std::endl;
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

    /// 5. Timer
    ros::Timer timer_pub = nh.createTimer(ros::Duration(2), funcPub, false);

    /// 6. spin
    ros::spin();

    ROS_INFO("Find Contact Points of Foot and Terrain Node Finished.");

    return 0;
}

void funcPub(const ros::TimerEvent &ev)
{
    pcl_conversions::toPCL(ros::Time::now(), cloud_bathy->header.stamp);
    pub_bathy.publish(cloud_bathy);
}

void funcSub_foot_contact_xyz(const std_msgs::Float64MultiArray::ConstPtr &input)
{
    if (input->data.size() == 18)
    {
        for (int foot_id = 0; foot_id < 6; foot_id++)
        {
            foot_point[foot_id].x       = input->data[3 * foot_id + 0];
            foot_point[foot_id].y       = input->data[3 * foot_id + 1];
            foot_point[foot_id].z_req   = input->data[3 * foot_id + 2];
        }

        // calculate Contact Z
        funcRealtimeRun();

        // publish
        // xyz
        array_contact_z.data.clear();
        for (int foot_id = 0; foot_id < 6; foot_id++)
        {
            array_contact_z.data.push_back(foot_point[foot_id].z);
        }
        pub_foot_contact.publish(array_contact_z);

        // cloud
        pcl_conversions::toPCL(ros::Time::now(), cloud_foot_contact->header.stamp);
        pub_foot_contact_cloud.publish(cloud_foot_contact);
    }
    else
    {
        ROS_WARN("Required number of Foot points is not 6.");
    }
}

void funcRealtimeRun(void)
{
    cloud_foot_contact->points.clear();
    cloud_foot_contact->header.frame_id = "world";

    int numberOfneighbor = 3;
    float searching_radius = 0.6;
    for (int search_foot_id = 0; search_foot_id < 6; search_foot_id++)
    {
        pcl::PointXYZ searchPoint;
        searchPoint.x = foot_point[search_foot_id].x;
        searchPoint.y = foot_point[search_foot_id].y;
        foot_point[search_foot_id].z = 999.9;

        // search z
        for (float search_z_position = -1.0; search_z_position < 0.5; search_z_position = search_z_position + 0.1)
        {
            searchPoint.z = foot_point[search_foot_id].z_req + search_z_position;

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
                    foot_point[search_foot_id].z = sum_z / (float)numberOfPoint;

                    // cloud_2dmbes
                    pcl::PointXYZ p;
                    p.x = foot_point[search_foot_id].x;
                    p.y = foot_point[search_foot_id].y;
                    p.z = sum_z / (float)numberOfPoint;
                    cloud_foot_contact->push_back(p);

                    break;
                }
            }
        }
    }
}