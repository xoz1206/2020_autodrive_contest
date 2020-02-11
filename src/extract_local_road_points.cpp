#include <iostream>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
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

/* road find header */
#include <geometry_msgs/PoseStamped.h>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <tf_conversions/tf_eigen.h>


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
            sub_points_raw = nh.subscribe("/points_raw", 1000, &Road::CallBack_points_raw, this);
            sub_road_map = nh.subscribe("/cloud_pcd", 1000, &Road::CallBack_road_map, this);
            sub_localizer_pose = nh.subscribe("/localizer_pose", 1000, &Road::CallBack_localizer_pose, this);

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
                vg.setLeafSize(1.0f,1.0f,1.0f);//set the voxel grid size //10cm
                vg.filter(*cloud_filtered);//create the filtering object

                for(size_t k = 0; k < cloud_filtered->points.size(); ++k)
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
                
                Eigen::Matrix3f rotation_mat_roll, rotation_mat_pitch, rotation_mat_yaw, rotation_mat;

                // roll
                rotation_mat_roll << 1, 0, 0,
                                     0, cos(roll), -sin(roll),
                                     0, sin(roll), cos(roll);
                // pitch
                rotation_mat_pitch << cos(pitch), 0, sin(pitch),
                                        0, 1, 0,
                                        -sin(pitch), 0, cos(pitch);
                // yaw
                rotation_mat_yaw << cos(yaw), -sin(yaw), 0,
                                    sin(yaw),  cos(yaw) , 0,
                                    0, 0, 1;

                // ratation_matrix
                rotation_mat = rotation_mat_yaw * rotation_mat_pitch * rotation_mat_roll;
                
                // 좌표 변환
                for(size_t i = 0; i < local_road.points.size(); i++)
                {
                    Eigen::Vector3f p(local_road.points[i].x - pose_point.x, local_road.points[i].y - pose_point.y, local_road.points[i].z - pose_point.z);
                    Eigen::Vector3f result;

                    // 좌표 변환은 역행렬로!!!
                    result = rotation_mat.inverse() * p;
                    
                    local_road.points[i].x = result[0];
                    local_road.points[i].y = result[1];
                    local_road.points[i].z = result[2];
                }
                    
                int count = 0;

                for(size_t i = 0; i < local_road.points.size(); ++i)
                {
                    for(size_t k = 0; k < cloud_filtered_points_raw->points.size(); ++k)
                    {
                        if(!(fabs(cloud_filtered_points_raw->points[k].x) < x_limit &&
                            fabs(cloud_filtered_points_raw->points[k].y) < y_limit))
                        {
                            cloud_filtered_points_raw->points.erase(cloud_filtered_points_raw->points.begin() + k);
                            k--;
                            continue;
                        }
                        
                        if(fabs(cloud_filtered_points_raw->points[k].x - local_road.points[i].x) < 1.0 &&
                            fabs(cloud_filtered_points_raw->points[k].y - local_road.points[i].y) < 1.0 &&
                            fabs(cloud_filtered_points_raw->points[k].z - local_road.points[i].z) < 1.0
                        )
                        {
                            cloud_filtered_points_raw->points.erase(cloud_filtered_points_raw->points.begin() + k);
                            k--;
                            count++;
                        }
                    }
                }
                
                cout << cloud_filtered_points_raw->points.size() << ", " << local_road.points.size() << "  erase points count : " << count << endl;

                // XYZI 형식이 필요함.
                for(size_t k = 0; k < cloud_filtered_points_raw->points.size(); ++k)
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
                new_points_raw.header.frame_id = "velodyne";
                pub_my_points_raw.publish(new_points_raw);

                //local_road_points
                sensor_msgs::PointCloud2 local_road_points;
                pcl::PointCloud<pcl::PointXYZI>::Ptr n_ptr(new pcl::PointCloud<pcl::PointXYZI>(local_road));
                pcl::toROSMsg(*n_ptr, local_road_points);

                local_road_points.header.frame_id = "velodyne";
                pub_local_road_points.publish(local_road_points);

                new_points.points.resize(0);
                local_road.points.resize(0);
            }
            else if(pose_point.x == 0.0 && pose_point.y == 0.0 && pose_point.z == 0.0 && check == true)
            {
                ROS_INFO("we can't get [ pose ] information");
            }
            else if(pose_point.x != 0.0 && pose_point.y != 0.0 && pose_point.z != 0.0 && check == false)
            {
                ROS_INFO("we can't get [ points_raw ] information");
            }
            else
            {
                ROS_INFO("we can't get [ pose, points_raw ] information");
            }
        }

        void CallBack_localizer_pose(const geometry_msgs::PoseStampedConstPtr &ptr)
        {
            pose_point.x = ptr->pose.position.x;
            pose_point.y = ptr->pose.position.y;
            pose_point.z = ptr->pose.position.z;
            tf2::Quaternion q(ptr->pose.orientation.x, ptr->pose.orientation.y, ptr->pose.orientation.z, ptr->pose.orientation.w);
            tf2::Matrix3x3 m(q);
            m.getRPY(roll, pitch, yaw);
        

            cout << "pose         -  x   : " << pose_point.x << ",  y : " << pose_point.y << " , z : " << pose_point.z << endl;
            cout << "orientation  - roll : " << roll << ", pitch : " << pitch << " , yaw : " << yaw << endl;
        }
        
        void CallBack_points_raw(const sensor_msgs::PointCloud2ConstPtr &ptr)
        {
            pcl::fromROSMsg(*ptr, scan_points_raw);
            pcl::VoxelGrid<pcl::PointXYZ> vg;

            vg.setInputCloud(scan_points_raw.makeShared());//scan PointCloud data copy
            vg.setLeafSize(1.0f,1.0f,1.0f);//set the voxel grid size //10cm
            vg.filter(*cloud_filtered_points_raw);//create the filtering object
            check = true;
            
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

        //find road 
        geometry_msgs::Point pose_point;
        Eigen::Affine3d  rotation_matrix_orientation;
        pcl::PointCloud<pcl::PointXYZ> scan, scan_points_raw; 
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, cloud_filtered_points_raw;
        pcl::PointCloud<pcl::PointXYZI> local_road, new_points;

        //Radius
        int radius;
        bool check;
        double roll, pitch, yaw;

};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "extract_local_road_points");
    Road r;
    ros::spin();
}

