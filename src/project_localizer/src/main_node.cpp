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

bool map_received = true;
bool init_pose_received = false; 
bool first_scan = true; 

std::vector<Vector2i> obstacles;
std::string filename;
float resolution;
float dmax;


void mapCallBack(const nav_msgs::OccupancyGrid& map){
    if(map_received == false){
        std::cerr << "Map received" << std::endl;
        
        

        
        
        map_received = true;
    }
    std::cerr << "Dmap ready" << std::endl;
}

void initCallBack(const geometry_msgs::PoseWithCovarianceStamped& init){
    if(map_received == true && init_pose_received == false){
        std::cerr << "Init pose received" << std::endl;

        // Isometry2f X=Eigen::Isometry2f::Identity();
        // float x = init.pose.pose.position.x;
        // float y = init.pose.pose.position.y;
        // float angle = init.pose.pose.orientation.w;
        
        // X.translation() << x, y;
        // X.linear()=Eigen::Rotation2Df(angle).matrix(); 

        // localizer.X=X;
        
        init_pose_received = true;
    }
}

void laserCallBack(const sensor_msgs::LaserScan& scan){
    if(map_received == true && init_pose_received == true){
        // prepare canvas for visualization
        Canvas canvas;
        const auto& distances = localizer.distances;

        // extract the obstacles from the gmap to use as measurements
        std::vector<Vector2f> obstacles;
        for (int r=0; r<grid_map.rows; ++r)
            for (int c=0; c<grid_map.cols; ++c)
            if (grid_map(r,c)<127) {
                Vector2f endpoint=grid_map.grid2world(Vector2f(c,r));
                if (endpoint.norm()<50)
                obstacles.push_back(endpoint);
            }
        
        // 3. sugar: add an obstacle image as alternative background
        Grid_<uint8_t> obstacle_image(distances.rows, distances.cols);
        obstacle_image.fill(0);
        obstacle_image.draw(canvas, false);
        for (const auto& m: obstacles) {
            Vector2f m_hat_grid=grid_map.world2grid(m);
            drawCircle(canvas, m_hat_grid, 3, 255);
        }
        // we draw with cv and get back the result, dirty...
        memcpy(&obstacle_image.cells[0], canvas.data, distances.rows*distances.cols);

        // now run the localizer
        Isometry2f X=Eigen::Isometry2f::Identity();
        X.linear()=Eigen::Rotation2Df(0.1).matrix();
        X.translation()<< 0.5, 0.5;
        localizer.X=X;
        bool show_obstacles=false;
        while (1) {
            if (show_obstacles)
            obstacle_image.draw(canvas, true);
            else
            distances.draw(canvas, true);
            for (const auto& m: obstacles) {
            Vector2f m_hat=localizer.X*m;
            Vector2f m_hat_grid=grid_map.world2grid(m_hat);
            drawCircle(canvas, m_hat_grid, 3, 255);
            }
            int key = showCanvas(canvas,0);
            if (key == 32) {
            show_obstacles = !show_obstacles;
            continue;
            }

            
            localizer.localize(obstacles, 1);
        }
    }
}

int main(int argc, char** argv){
    if (argc<4) {
        std::cout << "usage " << argv[0] << "<map_image> <resolution> <dmax>" << std::endl;
        return -1;
    }

    filename = argv[1];
    resolution = atof(argv[2]);
    dmax = atof(argv[3]);

    grid_map.loadFromImage(filename.c_str(), resolution);
    localizer.setMap(grid_map,dmax);

    //std::string map_topic_name = "/map";
    std::string init_topic_name = "/initialpose";
    std::string laser_topic_name = "/laser";

    ros::init(argc,argv,"publisher_node");
    //ros::NodeHandle n_map; 
    ros::NodeHandle n_init_pose;
    ros::NodeHandle n_laser_scan;

    //ros::Subscriber sub = n_map.subscribe<const nav_msgs::OccupancyGrid&>(map_topic_name, 10, mapCallBack);
    ros::Subscriber sub_init_pose = n_init_pose.subscribe<const geometry_msgs::PoseWithCovarianceStamped&>(init_topic_name, 10, initCallBack);
    ros::Subscriber sub_laser_scan = n_laser_scan.subscribe<const sensor_msgs::LaserScan&>(laser_topic_name, 10, laserCallBack);

    ros::spin();
    return 0;
} 