#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/grid_utils.hpp>
#include <common_utils/geometric/point.hpp>
#include <slam/mapping.hpp>
SensorModel::SensorModel(void)
:   ray_stride_(5)
{
}

double SensorModel::likelihood(const mbot_lcm_msgs::particle_t& sample, 
                               const mbot_lcm_msgs::lidar_t& scan, 
                               const OccupancyGrid& map)
{
    double likelihood = 0.0;
    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose, ray_stride_);
    
    // for(auto&& ray : movingScan)
    // {
    //     likelihood += scoreRaySimple(ray, map);
    // }
	
	likelihood = scoreScanBeamModel(movingScan, map);

	return likelihood;
}

double SensorModel::scoreScanBeamModel(const MovingLaserScan& movingScan, const OccupancyGrid& map){
    float maxRange = 5.0f;
    float stdDev = 4.f;

    float lnConstCoeff = log(1 / (stdDev * sqrt(2 * M_PI)));
    float rangeSim;
    float rangeDiff;
    float sumLogLikelihoods = 0.0f;
    int numLogLikelihoods = 0;
    int numInvalids = 0;
	for (auto&& ray : movingScan){
    	if (ray.range > maxRange) {
    		numInvalids += 1;
    		continue;
    	}
    	rangeSim = simulate_ray(
    			ray.origin,
    			ray.theta,
    			maxRange, 
				map);
    	if (rangeSim < 0) {
    		numInvalids += 1;
    		continue;
    	}
    	rangeDiff = ray.range - rangeSim;
    	sumLogLikelihoods += lnConstCoeff - 0.5 * pow(rangeDiff / stdDev, 2.0);
    	numLogLikelihoods += 1;
	}

    sumLogLikelihoods += numInvalids * sumLogLikelihoods / numLogLikelihoods;
    
	return exp(sumLogLikelihoods);
}

float SensorModel::simulate_ray(
		const Point<float>& origin,
		const float& theta, 
		const float& maxRange,
		const OccupancyGrid& map)
{
	// Returns -1.0 for out-of-map or out-of-range

	// Get all grid cells along an infinite length ray
	adjusted_ray_t rayMax = {
		origin,
		maxRange,
		theta};
	std::vector<Point<int>> rayCells = Mapping::bresenham(rayMax, map);

	// Filter by occupied cells, find closest occupied cell
	Point<int> originCell = global_position_to_grid_cell(origin, map);
	int minRangeCellsSquared = -1;
	int rangeCellsSquared;
	for (auto&& cell : rayCells) {
		if (map.isCellInGrid(cell.x, cell.y)) {
			if (map.logOdds(cell.x, cell.y) > 0) {
				rangeCellsSquared = pow(originCell.x - cell.x, 2) + pow(originCell.y - cell.y, 2);
				if ((minRangeCellsSquared < 0) || (rangeCellsSquared < minRangeCellsSquared)) {
					minRangeCellsSquared = rangeCellsSquared;
				}
			}
		}
	}

	// Calculate distance in meters to closest occupied cell
	float minRangeMeters =	-1.0;
	if (minRangeCellsSquared >= 0) {
		minRangeMeters = sqrt((float) minRangeCellsSquared) * map.metersPerCell();
	}
	return minRangeMeters;
}

double SensorModel::scoreRaySimple(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    Point<int> point = global_position_to_grid_cell(
        Point<float>(
            ray.origin.x + ray.range * std::cos(ray.theta),
            ray.origin.y + ray.range * std::sin(ray.theta)
        ), 
    map
    );
    
    if (map.isCellInGrid(point.x, point.y)){
        CellOdds cellOdds = map.logOdds(point.x, point.y);
        if (cellOdds > 0){
            return (double)cellOdds;
        } else {
            return 0.0;
        }
    }
    
}