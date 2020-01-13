#include<iostream>
#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<geometry_msgs/Point.h>
#include<vector>
#include<algorithm>
#include<cstdlib>
#include<cmath>
#include<ctime>
#include<visualization_msgs/MarkerArray.h>
//#include<visualization_msgs/Marker.h>
#include<pcl_ros/point_cloud.h>
#include<visualization_msgs/Marker.h>
#include<std_msgs/ColorRGBA.h>
#include"clustering/find_road_points.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/sample_consensus/sac_model_plane.h>
//Ransac
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace std;

class find_road_points{
    public:
        find_road_points() : count(0)
        {
            pub_forward_points = nh.advertise<visualization_msgs::Marker>("/plus_points", 10);
            pub2 = nh.advertise<visualization_msgs::Marker>("/minus_points", 10);
            pub3 = nh.advertise<sensor_msgs::PointCloud2>("/plane", 10);
            pub4 = nh.advertise<sensor_msgs::PointCloud2>("/projected_cloud", 10);

            sub = nh.subscribe("/points_raw", 100, &find_road_points::get_points_callback, this);
        }
        void get_points_callback(const sensor_msgs::PointCloud2ConstPtr &ptr)
        {
            sensor_msgs::PointCloud2 point_msg;

            //PointCloud data change toROSMsg
            pcl::fromROSMsg(*ptr, scan);
            pcl::fromROSMsg(*ptr, filterd_scan);
            filterd_scan.clear();

            pcl::VoxelGrid<pcl::PointXYZ> vg;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

            vg.setInputCloud(scan.makeShared());//scan PointCloud data copy
            vg.setLeafSize(0.15f,0.15f,0.15f);//set the voxel grid size //15cm
            vg.filter(*cloud_filtered);//create the filtering object

            geometry_msgs::Point p;
            geometry_msgs::Point under_p;
            count = 0;

            for(unsigned int k = 0; k < cloud_filtered->points.size(); ++k)
            {
                // 가장 먼 두점을 구하기 위해 추출.
                if(cloud_filtered->points[k].z > -0.2) continue;
                
                p.x = cloud_filtered->points[k].x;
                p.y = cloud_filtered->points[k].y;
                p.z = cloud_filtered->points[k].z;
                input_points.push_back(p);

                // 바닥면을 검출하기 위해 필요한 points 추출.
                if(fabs(cloud_filtered->points[k].y) > 1.0 ) continue;

                p.x = cloud_filtered->points[k].x;
                p.y = cloud_filtered->points[k].y;
                p.z = cloud_filtered->points[k].z;

                count++;
                vec_road_point.push_back(p); //z축이 0보다 작고 y값이 0.1 보다 작은 point들의 vector
            }
            for(int k = 0; k < vec_road_point.size(); ++k)
            {
                if(vec_road_point[k].x < 0)
                    minus_point.push_back(vec_road_point[k]);
                else
                    plus_point.push_back(vec_road_point[k]);
            }
            sort(minus_point.begin(), minus_point.end(), compare_x); // 정렬
            sort(plus_point.begin(), plus_point.end(), compare_x); // 정렬

            filitering(); // z축이 급격하게 변화하는 구간부터 삭제
 
            make_plane_jaemin_code();


            //make_plane_RANSAC();
            projection_onto_plane();

            
            // cout << "minus" << endl;
            // for(int k = 0; k < minus_point.size(); ++k)
            //     cout << minus_point[k].x << " " << minus_point[k].y << " " << minus_point[k].z << endl;
             
            // cout << "plus" << endl;
            // for(int k = 0; k < plus_point.size(); ++k)
            //     cout << plus_point[k].x << " " << plus_point[k].y << " " << plus_point[k].z << endl;
            
            
            print_rviz_marker_minus();
            print_rviz_marker_plus();

            cout << "count : " << count << "data size() : " << ptr->data.size() << endl;

            //초기화
            vec_road_point.resize(0);
            minus_point.resize(0);
            plus_point.resize(0);
        }

        // 재민 코드
        void make_plane_jaemin_code()
        {
            geometry_msgs::Point vector_OP; // 원점과 q_plus_avg
            geometry_msgs::Point vector_OQ; // 원점과 q_minus_max
            geometry_msgs::Point O; //원점

            vector< geometry_msgs::Point > vec_left;
            vector< geometry_msgs::Point > vec_right;

            //왼쪽 오른쪽 경사를 위해 앞의 두 점을 구한다. 가장 먼점과 같은 x축
            geometry_msgs::Point A_point;
            geometry_msgs::Point P_point;
            geometry_msgs::Point Q_point;

            A_point.x = plus_point[plus_point.size() - 2].x; //마지막에서 두번째 점.
            A_point.y = plus_point[plus_point.size() - 2].y;
            A_point.z = plus_point[plus_point.size() - 2].z;

            for(int k = 0; k < input_points.size(); ++k)
            {
                if(fabs(input_points[k].x - A_point.x) < 0.1  && fabs(input_points[k].z - A_point.z) < 0.05 && fabs(input_points[k].y) < 0.5)
                {
                    if(input_points[k].y < 0)
                        vec_left.push_back(input_points[k]);
                    else
                        vec_right.push_back(input_points[k]);
                }
            }
            // 아무런 정보를 얻지 못했을 때 이전에 구한 값으로 진행한다.
            if(vec_left.size() == 0)
                vec_left.push_back(prev_P_point);
            if(vec_right.size() == 0)
                vec_right.push_back(prev_Q_point);

            cout << "vec_left size() : " << vec_left.size() << "vec_right size() : " << vec_right.size() << endl;

            //앞의 왼쪽 평균, 오른쪽 평균을 구한다. 각각 P, Q가 된다.
            float x = 0.0;
            float y = 0.0;
            float z = 0.0;

            for(int k = 0; k<vec_left.size(); ++k)
            {
                x += vec_left[k].x;
                y += vec_left[k].y;
                z += vec_left[k].z;
            }
            P_point.x = x / vec_left.size();
            P_point.y = y / vec_left.size();
            P_point.z = z / vec_left.size();

            prev_P_point.x = x / vec_left.size();
            prev_P_point.y = y / vec_left.size();
            prev_P_point.z = z / vec_left.size();

            x = 0.0;
            y = 0.0;
            z = 0.0;

            for(int k = 0; k<vec_right.size(); ++k)
            {
                x += vec_right[k].x;
                y += vec_right[k].y;
                z += vec_right[k].z;
            }

            Q_point.x = x / vec_right.size();
            Q_point.y = y / vec_right.size();
            Q_point.z = z / vec_right.size();

            prev_Q_point.x = x / vec_right.size();
            prev_Q_point.y = y / vec_right.size();
            prev_Q_point.z = z / vec_right.size();

            // // plus 와 minus 축의 평균으로 계산한다. 완만한 변화를 위해..
            // x = 0.0;
            // y = 0.0;
            // z = 0.0;
            
            // for(int k = 0; k<plus_point.size(); ++k)
            // {
            //     x += plus_point[k].x;
            //     y += plus_point[k].y;
            //     z += plus_point[k].z;                
            // }

            // q_plus_max.x = x / plus_point.size();
            // q_plus_max.y = y / plus_point.size();
            // q_plus_max.z = z / plus_point.size();

            x = 0.0;
            y = 0.0;
            z = 0.0;

            for(int k = 0; k<minus_point.size(); ++k)
            {
                x += minus_point[k].x;
                y += minus_point[k].y;
                z += minus_point[k].z;                
            }
            
            q_minus_max.x = x / minus_point.size();
            q_minus_max.y = y / minus_point.size();
            q_minus_max.z = z / minus_point.size();

            O.x = Q_point.x; O.y = Q_point.y; O.z = Q_point.z;// 자동차의 가운데 지면과 닿는 점.
            
            //OP
            vector_OP.x = P_point.x - O.x;
            vector_OP.y = P_point.y - O.y;
            vector_OP.z = P_point.z - O.z;

            //OQ
            vector_OQ.x = q_minus_max.x - O.x;
            vector_OQ.y = q_minus_max.y - O.y;
            vector_OQ.z = q_minus_max.z - O.z;

            //OP x OQ
            normal_vector.x = vector_OP.y*vector_OQ.z - vector_OP.z*vector_OQ.y;
            normal_vector.y = vector_OP.z*vector_OQ.x - vector_OP.x*vector_OQ.z;
            normal_vector.z = vector_OP.x*vector_OQ.y - vector_OP.y*vector_OQ.x;
            //D = (P_point.x * (q_minus_max.y*O.z - O.y*q_minus_max.z) + q_minus_max.x*(O.y*P_point.z - P_point.y*O.z) + O.x*(q_plus_max.y*q_minus_max.z - q_minus_max.y*q_plus_max.z));
            float row_point_z = plus_point[plus_point.size() - 2].z;
            D = row_point_z * normal_vector.z; // 자동차의 가장 아랫점 (0,0,-1.785)를 지나기 때문에 그값을 평면의 방정식에 넣은 값.
            //법선벡터 출력
            cout << " -------------------------------- " << endl;
            cout << "normal vector : " << "( " << normal_vector.x << ", " << normal_vector.y << ", " << normal_vector.z << " )" << endl;
            cout << "any one point : " << "( " << O.x << ", " << O.y << ", " << O.z << " )" << endl;

            pcl::PointCloud<pcl::PointXYZI> TotalCloud;
            
            float z_ = 0.0;

            for (float x = -7.5; x < 7.5; x+=0.1)
            {
                for(float y = -5.0; y < 5.0; y+=0.1)
                {
                    z_ = (-1) * (normal_vector.x * x + normal_vector.y * y - D) / normal_vector.z;

                    pcl::PointXYZI pt2;
                    pt2.x = x;
                    pt2.y = y;
                    pt2.z = z_;
                    pt2.intensity = 1.0;

                    TotalCloud.push_back(pt2);
                }
            }

            sensor_msgs::PointCloud2 plane_point;
            pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(TotalCloud));
            pcl::toROSMsg(*scan_ptr, plane_point);

            plane_point.header.frame_id = "velodyne";
            pub3.publish(plane_point);

        }
        

        void projection_onto_plane()
        {
            Eigen::Vector4f coeffs;
            coeffs << normal_vector.x, normal_vector.y, normal_vector.z, -D;

            pcl::PointCloud<pcl::PointXYZI> projected_cloud_pcl;

            for(size_t i = 0; i < scan.points.size(); ++i)
            {
                // projection이 수행되어야 하는 영역안의 points 추출 후, projection
                if(fabs(scan.points[i].x) < x_limit && fabs(scan.points[i].y) < y_limit && scan.points[i].z < z_high_limit &&  scan.points[i].z > z_low_limit )
                {
                    pcl::PointXYZI projection;

                    projection.x = scan.points[i].x; 
                    projection.y = scan.points[i].y;
                    projection.z = (-1) * (normal_vector.x * scan.points[i].x + normal_vector.y * scan.points[i].y - D) / normal_vector.z;
                    
                    projection.intensity = 2.0;

                    projected_cloud_pcl.push_back(projection);
                }
            }

            sensor_msgs::PointCloud2 projected_cloud;
            pcl::PointCloud<pcl::PointXYZI>::Ptr n_ptr(new pcl::PointCloud<pcl::PointXYZI>(projected_cloud_pcl));
            pcl::toROSMsg(*n_ptr, projected_cloud);

            projected_cloud.header.frame_id = "velodyne";
            pub4.publish(projected_cloud);
        }

        void filitering()
        {
            for(int k = 0; k<minus_point.size()-1; ++k)
            {
                if(fabs(minus_point[k].z - minus_point[k+1].z) > z_range_of_change) // z축이 급격하게 변화한다면.
                {
                    for(int i = minus_point.size() - 1; i > k; --i) // 뒤에 전부 삭제
                    {
                        minus_point.erase(minus_point.begin() + i);
                    }
                }
            }

            for(int k = 1; k<plus_point.size()-1; ++k) // plus에는 (0,0,0)이 들어있다.
            {
                if(fabs(plus_point[k].z - plus_point[k+1].z) > z_range_of_change) // z축이 급격하게 변화한다면.
                {
                    for(int i = plus_point.size() - 1; i > k; --i) // 뒤에 전부 삭제
                    {
                        plus_point.erase(plus_point.begin() + i);
                    }
                }
            }
        }

        static bool compare_x(geometry_msgs::Point gm_1, geometry_msgs::Point gm_2)
        {
            if(fabs(gm_1.x) == fabs(gm_2.x))
            {
                if(fabs(gm_1.y) == fabs(gm_2.y))
                {
                    if(fabs(gm_1.z) == fabs(gm_2.z))
                        return true;
                    else if( fabs(gm_1.z) > fabs(gm_2.z))
                        return false;
                    else
                        return true;
                }
                else if(fabs(gm_1.y) > fabs(gm_2.y))
                    return false;
                else
                {
                    return true;
                }
            }
            else if(fabs(gm_1.x) > fabs(gm_2.x))
            {
                return false;
            }
            else
                return true;
        }

        void print_rviz_marker_minus()
        {
            visualization_msgs::Marker points;
            std_msgs::ColorRGBA c;

            int id = 0;
            c.r = 1.0;
            c.g = 0.3;
            c.b = 1.0;
            c.a = 1.0;

            for(int k = 0; k < minus_point.size(); ++k)
            {
                points.header.frame_id = "velodyne";
                points.header.stamp = ros::Time::now();
                points.ns = "points";
                points.action = visualization_msgs::Marker::ADD;
                points.pose.orientation.w = 1.0;
                points.lifetime = ros::Duration(0.1);
                points.id = id++;
                points.type = visualization_msgs::Marker::POINTS;
                points.scale.x = 0.1;
                points.scale.y = 0.1;

                points.color.r = 0.0;
                points.color.g = 0.0;
                points.color.b = 1.0;
                points.color.a = 1.0;

                points.points.push_back(minus_point[k]);
                points.colors.push_back(c);
            }
            
            pub_forward_points.publish(points);
            points.points.clear();
            points.colors.clear();
        }
        void print_rviz_marker_plus()
        {
            visualization_msgs::Marker points;
            std_msgs::ColorRGBA c;

            int id = 0;
            c.r = 0.0;
            c.g = 0.0;
            c.b = 1.0;
            c.a = 1.0;

            for(int k = 0; k < plus_point.size(); ++k)
            {
                points.header.frame_id = "velodyne";
                points.header.stamp = ros::Time::now();
                points.ns = "points";
                points.action = visualization_msgs::Marker::ADD;
                points.pose.orientation.w = 1.0;
                points.lifetime = ros::Duration(0.1);
                points.id = id++;
                points.type = visualization_msgs::Marker::POINTS;
                points.scale.x = 0.1;
                points.scale.y = 0.1;

                points.color.r = 0.0;
                points.color.g = 0.0;
                points.color.b = 1.0;
                points.color.a = 1.0;

                points.points.push_back(plus_point[k]);
                points.colors.push_back(c);
            }
            
            pub2.publish(points);
            points.points.clear();
            points.colors.clear();
        }

        
    private:
        ros::NodeHandle nh;
        ros::Publisher pub_forward_points;
        ros::Publisher pub2;
        ros::Publisher pub3;
        ros::Publisher pub4;

        ros::Subscriber sub;

        vector< geometry_msgs::Point > vec_road_point;
        vector< geometry_msgs::Point > minus_point;
        vector< geometry_msgs::Point > plus_point;
        vector< geometry_msgs::Point > before_projection_points;
        vector< geometry_msgs::Point > input_points;

        geometry_msgs::Point q_plus_max;
        geometry_msgs::Point q_minus_max;
        geometry_msgs::Point normal_vector; //법선벡터
        geometry_msgs::Point prev_Q_point;
        geometry_msgs::Point prev_P_point;

        pcl::PointCloud<pcl::PointXYZ> scan, filterd_scan;

        int count;
        float D; // 평면의 방정식의 상수 값
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "find_road_points");
    find_road_points frp;
    ros::spin();
}