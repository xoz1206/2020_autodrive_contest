#include <iostream>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <queue>

/* Quadtree header */
#include <vector>
#include <string>
#include <cmath>
#include <Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>
#include <ctime>
#include <cstdlib>
/* ascent header*/
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
/* user header */
#include "clustering/find_road_points.h"

using namespace std;
using namespace pcl;

class Road
{
    public:
        Road() : cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>), radius(40)
        {
            //Publisher
            pub_local_road_points = nh.advertise<sensor_msgs::PointCloud2>("/local_road_points", 1000);

            //Subscriber
            sub_road_map = nh.subscribe("/cloud_pcd", 10, &Road::CallBack_road_map, this);
            sub_localizer_pose = nh.subscribe("/localizer_pose", 10, &Road::CallBack_localizer_pose, this);
            
            pose_point.x = 0.0;
            pose_point.y = 0.0;
            pose_point.z = 0.0;
        }
        
        void CallBack_road_map(const sensor_msgs::PointCloud2ConstPtr &ptr)
        {
            if(pose_point.x != 0.0 && pose_point.y != 0.0 && pose_point.z != 0.0)
            {
                pcl::fromROSMsg(*ptr, scan);
                pcl::VoxelGrid<pcl::PointXYZ> vg;

                vg.setInputCloud(scan.makeShared());//scan PointCloud data copy
                vg.setLeafSize(0.15f,0.15f,0.15f);//set the voxel grid size //10cm
                vg.filter(*cloud_filtered);//create the filtering object

                
                // z 값이 filtering 된 point들을 가지고 pointcloud 만드는 작업. RANSAC 알고리즘에 넣어주기 위해
                for(int k = 0; k < cloud_filtered->points.size(); ++k)
                {
                    if(cloud_filtered->points[k].x < pose_point.x + radius &&
                        cloud_filtered->points[k].x > pose_point.x - radius &&
                        cloud_filtered->points[k].y < pose_point.y + radius &&
                        cloud_filtered->points[k].y > pose_point.y - radius)
                    {
                        pcl::PointXYZI road_point;

                        road_point.x = cloud_filtered->points[k].x;
                        road_point.y = cloud_filtered->points[k].y;
                        road_point.z = cloud_filtered->points[k].z;

                        pcl_scan.points.emplace_back(road_point);
                    }
                }

                sensor_msgs::PointCloud2 local_road_points;
                pcl::PointCloud<pcl::PointXYZI>::Ptr n_ptr(new pcl::PointCloud<pcl::PointXYZI>(pcl_scan));
                pcl::toROSMsg(*n_ptr, local_road_points);

                local_road_points.header.frame_id = "map";
                pub_local_road_points.publish(local_road_points);

                pcl_scan.points.resize(0);
            }
            else
            {
                ROS_INFO("we can't pose information");
            }

        }

        void CallBack_localizer_pose(const geometry_msgs::PoseStampedConstPtr &ptr)
        {
            pose_point.x = ptr->pose.position.x;
            pose_point.y = ptr->pose.position.y;
            pose_point.z = ptr->pose.position.z;
            cout << "x : " << pose_point.x << ",  y : " << pose_point.y << " , z : " << pose_point.z << endl;
        }
        
    private:
        ros::NodeHandle nh;
        //Publisher
        ros::Publisher pub_local_road_points;
        //Subscriber
        ros::Subscriber sub_road_map;
        ros::Subscriber sub_localizer_pose;

        geometry_msgs::Point pose_point;
        pcl::PointCloud<pcl::PointXYZ> scan; 
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;
        pcl::PointCloud<pcl::PointXYZI> pcl_scan;

        //Radius
        int radius;

};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "extract_local_road_points");
    Road r;
    ros::spin();
}

