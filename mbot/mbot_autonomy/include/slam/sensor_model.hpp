#ifndef SLAM_SENSOR_MODEL_HPP
#define SLAM_SENSOR_MODEL_HPP

class  lidar_t;
class  OccupancyGrid;
struct particle_t;


#include <common_utils/geometric/point.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <utils/grid_utils.hpp>
#include <list>
#include <numeric>
#include <random>


/**
* SensorModel implement a sensor model for computing the likelihood that a laser scan was measured from a
* provided pose, give a map of the environment.
* 
* A sensor model is compute the unnormalized likelihood of a particle in the proposal distribution.
*
* To use the SensorModel, a single method exists:
*
*   - double likelihood(const particle_t& particle, const lidar_t& scan, const OccupancyGrid& map)
*
* likelihood() computes the likelihood of the provided particle, given the most recent laser scan and map estimate.
*/
class SensorModel
{

    // using pose_offset_queue_t = std::queue<pose_offset_t>;
    using pose_offset_vector_t = std::vector<Point<int>>;

public:

    /**
    * Constructor for SensorModel.
    */
    SensorModel(void);

    /**
    * likelihood computes the likelihood of the provided particle, given the most recent laser scan and map estimate.
    * 
    * \param    particle            Particle for which the log-likelihood will be calculated
    * \param    scan                Laser scan to use for estimating log-likelihood
    * \param    map                 Current map of the environment
    * \return   Likelihood of the particle given the current map and laser scan.
    */
    double likelihood(const mbot_lcm_msgs::particle_t& particle,
                      const mbot_lcm_msgs::lidar_t& scan,
                      const OccupancyGrid& map);
    
    double scoreRaySimple(const adjusted_ray_t& ray, const OccupancyGrid& map);
    double scoreScanBeamModel(const MovingLaserScan& ray, const OccupancyGrid& map);
    
    float simulate_ray(
            const Point<float>& origin,
            const float& theta, 
            const float& maxRange,
            const OccupancyGrid& map);

    float max_scan_score;  // TODO: make getter

private:
    const int ray_stride_;
    ///////// TODO: Add any private members for your SensorModel ///////////////////
};

#endif // SLAM_SENSOR_MODEL_HPP
