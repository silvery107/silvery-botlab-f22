#include <planning/obstacle_distance_grid.hpp>
#include <slam/occupancy_grid.hpp>


ObstacleDistanceGrid::ObstacleDistanceGrid(void)
: width_(0)
, height_(0)
, metersPerCell_(0.05f)
, cellsPerMeter_(20.0f)
{
}

void ObstacleDistanceGrid::initializeDistances(const OccupancyGrid& map)
{
    int width = map.widthInCells();
    int height = map.heightInCells();
    cell_t cell;

    for (cell.y = 0; cell.y < height; ++cell.y)
    {
        for (cell.x = 0; cell.x < width; ++cell.x)
        {
            if (is_cell_free(cell, map))
            {
                // It's a flag so we know we haven't initialized distance
                distance(cell.x,cell.y) = -1.0;
            }
            else if (is_cell_occupied(cell, map))
            {
                distance(cell.x, cell.y) = 0.0;
            }
            else {
                distance(cell.x, cell.y) = 0.0;
            }
        }
        
    }
    
}


void ObstacleDistanceGrid::setDistances(const OccupancyGrid& map)
{
    resetGrid(map);
    
    ///////////// TODO: Implement an algorithm to mark the distance to the nearest obstacle for every cell in the map.
    initializeDistances(map);

    std::priority_queue<DistanceNode> searchQueue;
    enqueue_obstacle_cells(map, *this, searchQueue);

    while (!(searchQueue.empty()))
    {
        auto nextNode = searchQueue.top();
        searchQueue.pop();
        expand_node(nextNode, *this, searchQueue);
    }
}


bool ObstacleDistanceGrid::isCellInGrid(int x, int y) const
{
    return (x >= 0) && (x < width_) && (y >= 0) && (y < height_);
}


void ObstacleDistanceGrid::resetGrid(const OccupancyGrid& map)
{
    // Ensure the same cell sizes for both grid
    metersPerCell_ = map.metersPerCell();
    cellsPerMeter_ = map.cellsPerMeter();
    globalOrigin_ = map.originInGlobalFrame();
    
    // If the grid is already the correct size, nothing needs to be done
    if((width_ == map.widthInCells()) && (height_ == map.heightInCells()))
    {
        return;
    }
    
    // Otherwise, resize the vector that is storing the data
    width_ = map.widthInCells();
    height_ = map.heightInCells();
    
    cells_.resize(width_ * height_);
}

void enqueue_obstacle_cells(const OccupancyGrid& map, 
                                ObstacleDistanceGrid& grid, 
                                std::priority_queue<DistanceNode>& search_queue)
{
    int width = map.widthInCells();
    int height = map.heightInCells();
    cell_t cell;

    for (cell.y = 0; cell.y < height; ++cell.y)
    {
        for (cell.x = 0; cell.x < width; ++cell.x)
        {
            if (is_cell_occupied(cell, map))
            {
                expand_node(DistanceNode(cell, 0), grid, search_queue);
            }
        }
    }
}

void expand_node(const DistanceNode& node, ObstacleDistanceGrid& grid, std::priority_queue<DistanceNode>& search_queue)
{
    // TODO: Expand to 8 way? yeees!
    const int xDeltas[8] = {1, -1, 0, 0, 1, -1, 1, -1};
    const int yDeltas[8] = {0, 0, 1, -1, 1, -1, -1, 1};

    for (int n = 0; n < 8; ++n)
    {
        cell_t adjacentCell(node.cell.x + xDeltas[n], node.cell.y + yDeltas[n]);
        if (grid.isCellInGrid(adjacentCell.x, adjacentCell.y))
        {
            // Not seen yet
            if (grid(adjacentCell.x, adjacentCell.y) < 0)
            {
                float distance = node.distance;
                if (n < 4) distance += 1.0;
                else distance += 1.414;
                DistanceNode adjacentNode(adjacentCell, distance);
                grid(adjacentCell.x, adjacentCell.y) = adjacentNode.distance * grid.metersPerCell();
                search_queue.push(adjacentNode);
            }
        }
    }
}

bool is_cell_free(cell_t cell, const OccupancyGrid& map)
{
    return map.logOdds(cell.x, cell.y) < 0;
}

bool is_cell_occupied(cell_t cell, const OccupancyGrid& map)
{
    return map.logOdds(cell.x, cell.y) >= 0;
}
