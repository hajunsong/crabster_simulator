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
ros::Publisher pub_track_contact_cloud;
ros::Publisher pub_track_contact;
ros::Publisher pub_track_contact_dyn2;
ros::Subscriber sub_track_contact_xyz;
ros::Subscriber sub_track_contact_xyz_dyn2;

// Cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bathy(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_track_contact(new pcl::PointCloud<pcl::PointXYZ>);

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

struct struct_track_point
{
    struct_point point[8];
};

struct_track_point track[4];

void funcPub(const ros::TimerEvent &ev);
void funcSub_track_contact_xyz(const std_msgs::Float64MultiArray::ConstPtr &input);
void funcSub_track_contact_xyz_dyn2(const std_msgs::Float64MultiArray::ConstPtr &input);
void funcRealtimeRun(void);

int main(int argc, char *argv[])
{
    ROS_INFO("Find Contact Points of Track and Terrain Node");
    /// 1. Init
    ros::init(argc, argv, "contact_track_and_terrain_node");
    ros::NodeHandle nh;

    /// 3. Publish & Subscriber
    pub_bathy = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("jeju_terrain/cloud", 10);
    pub_track_contact_cloud = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("rover/track/contact_cloud", 10);
    pub_track_contact = nh.advertise<std_msgs::Float64MultiArray>("rover/track/contact_z", 5);
    pub_track_contact_dyn2 = nh.advertise<std_msgs::Float64MultiArray>("dyn2/rover/track/contact_z", 5);

    sub_track_contact_xyz = nh.subscribe<std_msgs::Float64MultiArray>("rover/track/node_xyz", 5, &funcSub_track_contact_xyz);
    sub_track_contact_xyz_dyn2 = nh.subscribe<std_msgs::Float64MultiArray>("dyn2/rover/track/node_xyz", 5, &funcSub_track_contact_xyz_dyn2);

    // 4. Load file
    /// 4-1. Map file
    // std::string filePath_bathy = ros::package::getPath("cpos_maphandler_dynamics") + "/files/soilbasin_jeju_terrain_v5.ply";
    std::string filePath_bathy = ros::package::getPath("cpos_maphandler_dynamics") + "/files/jeju_bathymetry_soilbasin_frame_ascii.ply";
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
        cloud_bathy->header.frame_id = "Ground";

        // set searching target
        kdtree.setInputCloud(cloud_bathy);
    }

    /// 5. Timer
    ros::Timer timer_pub = nh.createTimer(ros::Duration(2), funcPub, false);

    /// 6. spin
    ros::spin();

    ROS_INFO("Find Contact Points of Track and Terrain Node Finished.");

    return 0;
}

void funcPub(const ros::TimerEvent &ev)
{
    pcl_conversions::toPCL(ros::Time::now(), cloud_bathy->header.stamp);
    pub_bathy.publish(cloud_bathy);
}

void funcSub_track_contact_xyz(const std_msgs::Float64MultiArray::ConstPtr &input)
{
    if (input->data.size() == 96)
    {
        for (int track_id = 0; track_id < 4; track_id++)
        {
            for (int point_id = 0; point_id < 8; point_id++)
            {
                track[track_id].point[point_id].x = input->data[24 * track_id + 3 * point_id + 0];
                track[track_id].point[point_id].y = input->data[24 * track_id + 3 * point_id + 1];
                track[track_id].point[point_id].z_req = input->data[24 * track_id + 3 * point_id + 2];
            }
        }

        // calculate Contact Z
        funcRealtimeRun();

        // publish
        // xyz
        array_contact_z.data.clear();
        for (int track_id = 0; track_id < 4; track_id++)
        {
            for (int point_id = 0; point_id < 8; point_id++)
            {
                array_contact_z.data.push_back(track[track_id].point[point_id].z);
            }
        }
        pub_track_contact.publish(array_contact_z);

        // cloud
        pcl_conversions::toPCL(ros::Time::now(), cloud_track_contact->header.stamp);
        pub_track_contact_cloud.publish(cloud_track_contact);
    }
    else
    {
    }
}

void funcSub_track_contact_xyz_dyn2(const std_msgs::Float64MultiArray::ConstPtr &input)
{
    if (input->data.size() == 96)
    {
        for (int track_id = 0; track_id < 4; track_id++)
        {
            for (int point_id = 0; point_id < 8; point_id++)
            {
                track[track_id].point[point_id].x = input->data[24 * track_id + 3 * point_id + 0];
                track[track_id].point[point_id].y = input->data[24 * track_id + 3 * point_id + 1];
                track[track_id].point[point_id].z_req = input->data[24 * track_id + 3 * point_id + 2];
            }
        }

        // calculate Contact Z
        funcRealtimeRun();

        // publish
        // xyz
        array_contact_z.data.clear();
        for (int track_id = 0; track_id < 4; track_id++)
        {
            for (int point_id = 0; point_id < 8; point_id++)
            {
                array_contact_z.data.push_back(track[track_id].point[point_id].z);
            }
        }
        pub_track_contact_dyn2.publish(array_contact_z);

        // cloud
        pcl_conversions::toPCL(ros::Time::now(), cloud_track_contact->header.stamp);
        pub_track_contact_cloud.publish(cloud_track_contact);
    }
    else
    {
    }
}

void funcRealtimeRun(void)
{
    cloud_track_contact->points.clear();
    cloud_track_contact->header.frame_id = "Ground";

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
            for (float search_z_position = -1.0; search_z_position < 0.5; search_z_position = search_z_position + 0.1)
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
}
