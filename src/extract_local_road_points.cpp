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
        Road() : cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>), cloud_filtered_points_raw(new pcl::PointCloud<pcl::PointXYZ>), radius(40), check(false)
        {
            //Publisher
            pub_local_road_points = nh.advertise<sensor_msgs::PointCloud2>("/local_road_points", 1000);
            pub_my_points_raw = nh.advertise<sensor_msgs::PointCloud2>("/my_points_raw", 1000);
            //Subscriber
            sub_points_raw = nh.subscribe("/points_raw", 10, &Road::CallBack_points_raw, this);
            sub_road_map = nh.subscribe("/cloud_pcd", 10, &Road::CallBack_road_map, this);
            sub_localizer_pose = nh.subscribe("/localizer_pose", 10, &Road::CallBack_localizer_pose, this);
            
            pose_point.x = 0.0;
            pose_point.y = 0.0;
            pose_point.z = 0.0;
        }
        
        void CallBack_road_map(const sensor_msgs::PointCloud2ConstPtr &ptr)
        {
            if(pose_point.x != 0.0 && pose_point.y != 0.0 && pose_point.z != 0.0 && check != false)
            {
                pcl::fromROSMsg(*ptr, scan);
                pcl::VoxelGrid<pcl::PointXYZ> vg;

                vg.setInputCloud(scan.makeShared());//scan PointCloud data copy
                vg.setLeafSize(0.15f,0.15f,0.15f);//set the voxel grid size //10cm
                vg.filter(*cloud_filtered);//create the filtering object

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

                        local_road.points.emplace_back(road_point);
                    }
                }

                Eigen::Matrix3f rotation_mat_z, rotation_mat_x;
                rotation_mat_z << 0, -1, 0,
                                 -1, 0, 0,
                                  0, 0, 1;
                // 좌표 변환
                for(int i = 0; i < local_road.points.size(); i++)
                {
                    Eigen::Vector3f p(local_road.points[i].x- pose_point.x, local_road.points[i].y- pose_point.y, local_road.points[i].z -pose_point.z);
                    Eigen::Vector3f result;
                    result = rotation_mat_z * p;
                    local_road.points[i].x = -result[0] ;
                    local_road.points[i].y = result[1] ;
                    local_road.points[i].z = result[2] ;
                }

                sort(cloud_filtered_points_raw->points.begin(), cloud_filtered_points_raw->points.end(), compare_PointXYZ);
                sort(local_road.points.begin(), local_road.points.end(), compare_PointXYZI);
                    
                int index = 0;
                int count = 0;

                for(int k = 0; k < cloud_filtered_points_raw->points.size(); ++k)
                {
                    if(fabs(cloud_filtered_points_raw->points[k].x - local_road.points[index].x) < 1 &&
                        fabs(cloud_filtered_points_raw->points[k].y - local_road.points[index].y) < 1 &&
                        fabs(cloud_filtered_points_raw->points[k].z - local_road.points[index].z) < 1
                    )
                    {
                        cloud_filtered_points_raw->points.erase(cloud_filtered_points_raw->points.begin() + k);
                        k--;
                        index++;
                        count++;
                    }
                    else if(cloud_filtered_points_raw->points[k].x > local_road.points[index].x)
                    {
                        index++;
                    }
                }
                
                cout << cloud_filtered_points_raw->points.size() << ", " << local_road.points.size() << "  erase points count : " << count << endl;

                for(int k = 0; k < cloud_filtered_points_raw->points.size(); ++k)
                {
                    pcl::PointXYZI point;

                    point.x = cloud_filtered_points_raw->points[k].x;
                    point.y = cloud_filtered_points_raw->points[k].y;
                    point.z = cloud_filtered_points_raw->points[k].z;

                    new_points.points.emplace_back(point);
                }

                //new_points_raw
                sensor_msgs::PointCloud2 new_points_raw;
                pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(new_points));
                pcl::toROSMsg(*scan_ptr, new_points_raw);
                new_points_raw.header.frame_id = "map";
                pub_my_points_raw.publish(new_points_raw);

                //local_road_points
                sensor_msgs::PointCloud2 local_road_points;
                pcl::PointCloud<pcl::PointXYZI>::Ptr n_ptr(new pcl::PointCloud<pcl::PointXYZI>(local_road));
                pcl::toROSMsg(*n_ptr, local_road_points);

                local_road_points.header.frame_id = "map";
                pub_local_road_points.publish(local_road_points);

                new_points.points.resize(0);
                local_road.points.resize(0);
            }
            else if(pose_point.x == 0.0 && pose_point.y == 0.0 && pose_point.z == 0.0 && check == true)
            {
                ROS_INFO("we can't get pose information");
            }
            else if(pose_point.x != 0.0 && pose_point.y != 0.0 && pose_point.z != 0.0 && check == false)
            {
                ROS_INFO("we can't get points_raw information");
            }
            else
            {
                ROS_INFO("we can't get pose, points_raw information");
            }
        }

        void CallBack_localizer_pose(const geometry_msgs::PoseStampedConstPtr &ptr)
        {
            pose_point.x = ptr->pose.position.x;
            pose_point.y = ptr->pose.position.y;
            pose_point.z = ptr->pose.position.z;
            cout << "x : " << pose_point.x << ",  y : " << pose_point.y << " , z : " << pose_point.z << endl;
        }
        
        void CallBack_points_raw(const sensor_msgs::PointCloud2ConstPtr &ptr)
        {
            pcl::fromROSMsg(*ptr, scan_points_raw);
            pcl::VoxelGrid<pcl::PointXYZ> vg;

            vg.setInputCloud(scan_points_raw.makeShared());//scan PointCloud data copy
            vg.setLeafSize(0.15f,0.15f,0.15f);//set the voxel grid size //10cm
            vg.filter(*cloud_filtered_points_raw);//create the filtering object
            check = true;
            
        }

        static bool compare_PointXYZ(pcl::PointXYZ p1, pcl::PointXYZ p2)
        {
            if(p1.x < p2.x)
            {
                return true;
            }
            else if(p1.x > p2.x)
            {
                return false;
            }    
            else
            {
                if(p1.y < p2.y)
                {
                    return true;
                }
                else if(p1.y > p2.y)
                {
                    return false;
                }
                else
                {
                    if(p1.z < p2.z)
                    {
                        return true;
                    }
                    else if(p1.z > p2.z)
                    {
                        return false;
                    }
                    else
                    {
                        return true;
                    }
                }
            }
        }
        
        static bool compare_PointXYZI(pcl::PointXYZI p1, pcl::PointXYZI p2)
        {
            if(p1.x < p2.x)
            {
                return true;
            }
            else if(p1.x > p2.x)
            {
                return false;
            }    
            else
            {
                if(p1.y < p2.y)
                {
                    return true;
                }
                else if(p1.y > p2.y)
                {
                    return false;
                }
                else
                {
                    if(p1.z < p2.z)
                    {
                        return true;
                    }
                    else if(p1.z > p2.z)
                    {
                        return false;
                    }
                    else
                    {
                        return true;
                    }
                }
            }
        }
    private:
        ros::NodeHandle nh;
        //Publisher
        ros::Publisher pub_local_road_points;
        ros::Publisher pub_my_points_raw;
        //Subscriber
        ros::Subscriber sub_road_map;
        ros::Subscriber sub_localizer_pose;
        ros::Subscriber sub_points_raw;

        geometry_msgs::Point pose_point;
        pcl::PointCloud<pcl::PointXYZ> scan, scan_points_raw; 
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, cloud_filtered_points_raw;
        pcl::PointCloud<pcl::PointXYZI> local_road, new_points;

        //Radius
        int radius;
        bool check;

};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "extract_local_road_points");
    Road r;
    ros::spin();
}

