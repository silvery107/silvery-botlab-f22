#include <slam/mapping.hpp>
#include <utils/grid_utils.hpp>
#include <numeric>
#include <chrono> 
using namespace std::chrono; 

Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
, initialized_(false)
{
}


void Mapping::updateMap(const mbot_lcm_msgs::lidar_t& scan,
                        const mbot_lcm_msgs::pose_xyt_t& pose,
                        OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////
    if (!initialized_)
    {
        previousPose_ = pose;
    }
    initialized_ = true;

    MovingLaserScan adjustedrays(scan, previousPose_, pose,1);
    for(auto& ray : adjustedrays){
        scoreEndpoint(ray, map);
    }
    for(auto& ray : adjustedrays){
        scoreRay(ray, map);
    }
    previousPose_ = pose;
}

void Mapping::scoreEndpoint(const adjusted_ray_t& ray, OccupancyGrid& map)
{
//////////////// TODO: Implement your endpoint score ///////////////////////
    if (ray.range > kMaxLaserDistance_){
        return;
    }else{
        Point<int> point = global_position_to_grid_cell(
            Point<float>(
                ray.origin.x + ray.range * std::cos(ray.theta),
                ray.origin.y + ray.range * std::sin(ray.theta)
            ), 
        map
        );
        // Increase cell logodds
        if (map.isCellInGrid(point.x,point.y)){
            CellOdds prevOdds = map.logOdds(point.x, point.y);
            if ((prevOdds + kHitOdds_) >= 127) {
                map.setLogOdds(point.x, point.y, 127);
            } else {
                map.setLogOdds(point.x, point.y, prevOdds + kHitOdds_);
            }
        }
    }

}

void Mapping::scoreRay(const adjusted_ray_t& ray, OccupancyGrid& map)
{
//////////////// TODO: Implement your ray score ///////////////////////
    adjusted_ray_t rayCapped = {
        ray.origin,
        std::min(ray.range,kMaxLaserDistance_),
        ray.theta
    };
    std::vector<Point<int>> rayPoints = bresenham(rayCapped, map);
    for (auto& point : rayPoints){
        // Decrease cell logodds
        if (map.isCellInGrid(point.x, point.y)){
            CellOdds prevOdds = map.logOdds(point.x, point.y);
            if (prevOdds - kMissOdds_ <= -127) {
                map.setLogOdds(point.x, point.y, -127);
            } else {
                map.setLogOdds(point.x, point.y, prevOdds - kMissOdds_);
            }
        }
    }
}

/*
Takes the ray and map, and returns a vector of map cells to check
*/
std::vector<Point<int>> Mapping::bresenham(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    // Get global positions 
    Point<float> f_end = global_position_to_grid_position(
        Point<float>(
            ray.origin.x + ray.range * std::cos(ray.theta),
            ray.origin.y + ray.range * std::sin(ray.theta)
            ), 
        map
        );

    // Cells
    Point<int> start_cell = global_position_to_grid_cell(ray.origin, map);
    Point<int> end_cell;
    end_cell.x = static_cast<int>(f_end.x);
    end_cell.y = static_cast<int>(f_end.y);
    std::vector<Point<int>> cells_touched;
    //////////////// TODO: Implement Bresenham's Algorithm ////////////////

    int dx = abs(end_cell.x - start_cell.x);
    int dy = abs(end_cell.y - start_cell.y);
    int sx = start_cell.x < end_cell.x ? 1 : -1;
    int sy = start_cell.y < end_cell.y ? 1 : -1;
    int err = dx - dy;
    int x = start_cell.x;
    int y = start_cell.y;
    while(x != end_cell.x || y != end_cell.y){
        cells_touched.push_back(Point<int>(x, y));
        float e2 = 2*err;
        if (e2 >= -dy){
            err -= dy;
            x += sx;
        }
        if (e2 <= dx){
            err += dx;
            y += sy;
        }
    }

    return cells_touched;
}

std::vector<Point<int>> Mapping::divideAndStepAlongRay(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    auto end_cell = global_position_to_grid_cell(Point<double>(
        ray.origin.x + ray.range * std::cos(ray.theta),
        ray.origin.y + ray.range * std::sin(ray.theta)
        ), map);
    //////////////// TODO: Implement divide and step ////////////////
    std::vector<Point<int>> cells_touched;
    return cells_touched;
}
