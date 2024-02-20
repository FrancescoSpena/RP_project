#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <fstream>

#include <nav_msgs/OccupancyGrid.h> 
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <rp_stuff/dmap.h>
#include <rp_stuff/grid_map.h>
#include <rp_stuff/dmap_localizer.h>


DMap dmap(0,0);
DMapLocalizer localizer;
GridMap grid_map(0.1,1,1);

bool map_received = false;
bool init_pose_received = false; 
bool first_scan = true; 


void mapCallBack(const nav_msgs::OccupancyGrid& map){
    if(map_received == false){
        std::cerr << "Map received" << std::endl;
        std::vector<Vector2i> obstacles;
        
        uint32_t width = map.info.width;
        uint32_t height = map.info.height;
        float resolution = map.info.resolution;

        std::cerr << "    width = " << width << std::endl << "    height = " << height << std::endl;
        
        grid_map.resize(height,width);
        grid_map.cells = map.data;

        // Canvas canvas;
        // grid_map.draw(canvas);
        // showCanvas(canvas,0);

        // //Extract a obstacles points
        int i = 0;
        for(int j=0; j < grid_map.cells.size(); j++){
            if(grid_map.cells[j] == 100){
                //Convert the occupated cell in grid coordinates 
                Vector2i index = grid_map.ptr2idx(&grid_map.cells[j]);
                //Add the obstacles to list
                obstacles.push_back(index);
                ++i;
            }
        }

        std::cerr << "size obs = " << i << std::endl;

        int dmax_2 = pow(1/resolution,2);
        DMap dmap(grid_map.rows,grid_map.cols); 
        dmap.clear();
        dmap.compute(obstacles,dmax_2);

        // std::cerr << "num obstacles = " << obstacles.size() << std::endl;

        // if(obstacles.size() <= 0){
        //     std::cerr << "Not obstacles" << std::endl;
        //     return; 
        // }

        
        dmap.copyTo(localizer.distances,dmax_2);
        for(auto& f: localizer.distances.cells){
            f=sqrt(f)*grid_map.resolution();
        }

        localizer.distances.colDerivative(localizer.distances_dc);
        localizer.distances.rowDerivative(localizer.distances_dr);


        std::cerr << "localizer ready" << std::endl;
        std::cerr << "  rows:  " << localizer.distances.rows << std::endl << "  cols: " << localizer.distances.cols << std::endl;

        
        // Canvas canvas;
        // const auto& distances = localizer.distances;
        // Grid_<uint8_t> obstacle_image(distances.rows, distances.cols);
        
    
        // float f_min=1e9;
        // float f_max=0;
        // for(auto& f: distances.cells) {
        //     f_min=std::min(f, f_min);
        //     f_max=std::max(f, f_max);
        // }
        // float scale=255./(f_max-f_min);

        // // 2. copy the (normalized) distances
        // for (size_t i=0; i<distances.cells.size(); ++i) {
        //     obstacle_image.cells[i]=scale  * (distances.cells[i] - f_min);
        // }
        
        
        // obstacle_image.draw(canvas);
        // showCanvas(canvas,0);
        
        map_received = true;
    }
    std::cerr << "Dmap ready" << std::endl;
}

void initCallBack(const geometry_msgs::PoseWithCovarianceStamped& init){
    if(map_received == true && init_pose_received == false){
        std::cerr << "Init pose received" << std::endl;

        Isometry2f X=Eigen::Isometry2f::Identity();
        float x = init.pose.pose.position.x;
        float y = init.pose.pose.position.y;
        float angle = init.pose.pose.orientation.w;
        
        X.translation() << x, y;
        X.linear()=Eigen::Rotation2Df(angle).matrix(); 

        localizer.X=X;
        
        init_pose_received = true;
    }
}

void laserCallBack(const sensor_msgs::LaserScan& scan){
    if(map_received == true && init_pose_received == true){
        std::cerr << "Scan received" << std::endl;
        std::vector<Vector2f> scan_endpoints; 
        for(size_t i=0; i < scan.ranges.size(); ++i){
            float alpha = scan.angle_min + i * scan.angle_increment; 
            float r = scan.ranges[i];
            if(r < scan.range_min || r > scan.range_max)
                continue;
            scan_endpoints.push_back(Vector2f(r*cos(alpha),r*sin(alpha)));
        }

        Canvas canvas;
        float angle = scan.angle_min; 
        localizer.localize(scan_endpoints,10);
        grid_map.draw(canvas);
        localizer.distances.draw(canvas,true);
        for(const auto& ep: scan_endpoints){
            drawCircle(canvas, grid_map.world2grid(localizer.X*ep), 3, 255);
        }
        showCanvas(canvas,1);
    }
    return;
}

int main(int argc, char** argv){
    std::string map_topic_name = "/map";
    std::string init_topic_name = "/initialpose";
    std::string laser_topic_name = "/laser";

    ros::init(argc,argv,"publisher_node");
    ros::NodeHandle n_map; 
    ros::NodeHandle n_init_pose;
    ros::NodeHandle n_laser_scan;

    ros::Subscriber sub = n_map.subscribe<const nav_msgs::OccupancyGrid&>(map_topic_name, 10, mapCallBack);
    ros::Subscriber sub_init_pose = n_init_pose.subscribe<const geometry_msgs::PoseWithCovarianceStamped&>(init_topic_name, 10, initCallBack);
    ros::Subscriber sub_laser_scan = n_laser_scan.subscribe<const sensor_msgs::LaserScan&>(laser_topic_name, 10, laserCallBack);

    ros::spin();
    return 0;
} 