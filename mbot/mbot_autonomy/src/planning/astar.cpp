#include <common_utils/timestamp.h>
#include <planning/astar.hpp>
#include <algorithm>
#include <chrono>
const double SQ_2 = 1.414213562373095;
using namespace std::chrono;

mbot_lcm_msgs::robot_path_t search_for_path(mbot_lcm_msgs::pose_xyt_t start,
                                             mbot_lcm_msgs::pose_xyt_t goal,
                                             const ObstacleDistanceGrid& distances,
                                             const SearchParams& params)
{
    cell_t goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances);
     ////////////////// TODO: Implement your A* search here //////////////////////////
    mbot_lcm_msgs::robot_path_t path;

    bool path_found = false;
    Point<int> start_cell = global_position_to_grid_cell(Point<double>(start.x, start.y), distances);
    Point<int> goal_cell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances);
    Node start_node(start_cell.x, start_cell.y);
    Node goal_node(goal_cell.x, goal_cell.y);

    // printf("Get start node (%d, %d), goal node (%d, %d)\n", start_cell.x, start_cell.y, goal_cell.x, goal_cell.y);

    if (start_node==goal_node){
        path.path.push_back(start);
        path.path_length = path.path.size();
        path.utime = utime_now();
        return path;
    }

    start_node.h_cost = h_cost(&start_node, &goal_node, distances);
    start_node.g_cost = 0.0;
    PriorityQueue open_set;
    std::vector<Node*> closed_set;
    open_set.push(&start_node);
    while (!open_set.empty() || open_set.Q.size()>10000){
        auto cur_node = open_set.pop();
        // printf("Open set size: %d\n", open_set.Q.size());
        // printf("Get cur node (%d, %d)\n", cur_node->cell.x, cur_node->cell.y);
        // if (*cur_node == goal_node){
        if ((*cur_node).cell == goal_node.cell){
            std::vector<Node*> temp_path = extract_node_path(cur_node, &start_node);
            path.path = extract_pose_path(temp_path, distances);
            path_found = true;
            break;
        }
        std::vector<Node*> successors = expand_node(cur_node, distances, params);
        
        for (auto & node : successors){
            double temp_g_cost = cur_node->g_cost + g_cost(cur_node, node, distances, params);
            double node_g_cost = node->g_cost;
            Node *node_in_open = open_set.get_member(node);
            if (node_in_open != NULL){
                node_g_cost = node_in_open->g_cost;
            }
            if (temp_g_cost < node_g_cost){
                if (node_in_open != NULL){
                    node_in_open->parent = cur_node;
                    node_in_open->g_cost = temp_g_cost;
                } else{
                    node->parent = cur_node;
                    node->g_cost = temp_g_cost;
                    node->h_cost = h_cost(node, &goal_node, distances);
                    if (!is_in_list(node, closed_set))
                        open_set.push(node);
                    else
                        delete node;
                }
            }
        }
        closed_set.push_back(cur_node);
    }

    // printf("Closed set size: %d\n", closed_set.size());
    for (int i=closed_set.size()-1; i>0; i--)
    {
        delete closed_set[i];
    }

    if (!path_found){
        path.path.clear();
        path.path.push_back(start);
        path.path_length = path.path.size();
        path.utime = utime_now();
        return path;
    }

    mbot_lcm_msgs::robot_path_t pathFiltered;
    pathFiltered.path.push_back(path.path[0]);
    pathFiltered.path.push_back(path.path[1]);
    float currXDir = path.path[2].x - path.path[1].x;
    int currXDirSign = int(currXDir > 0) - int(currXDir < 0);
    float currYDir = path.path[2].y - path.path[1].y;
    int currYDirSign = int(currYDir > 0) - int(currYDir < 0);
    float nextXDir;
    int nextXDirSign;
    float nextYDir;
    int nextYDirSign;
    for (int i = 2; i < path.path.size() - 1; i++) {
        nextXDir = path.path[i+1].x - path.path[i].x;
        nextXDirSign = int(nextXDir > 0) - int(nextXDir < 0);
        nextYDir = path.path[i+1].y - path.path[i].y;
        nextYDirSign = int(nextYDir > 0) - int(nextYDir < 0);
        if (not ((nextXDirSign == currXDirSign) and (nextYDirSign == currYDirSign))) {
            pathFiltered.path.push_back(path.path[i]);
            currXDirSign = nextXDirSign;
            currYDirSign = nextYDirSign;
        }
    }
    pathFiltered.path.push_back(path.path[path.path.size() - 1]);
    float theta_x = pathFiltered.path.back().x - pathFiltered.path[pathFiltered.path.size() -2].x;
    float theta_y = pathFiltered.path.back().y - pathFiltered.path[pathFiltered.path.size() -2].y;
    pathFiltered.path.back().theta = std::atan2(theta_y, theta_x);
    pathFiltered.path_length = pathFiltered.path.size();
    pathFiltered.utime = utime_now();
    return pathFiltered;
}



double h_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances)
{
    // TODO: Return calculated h cost
    double h_cost = 0;
    int dx = abs(from->cell.x - goal->cell.x);
    int dy = abs(from->cell.y - goal->cell.y);
    
    h_cost = (dx + dy) + (SQ_2 - 2) * std::min(dx, dy);
    return h_cost;
}
double g_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    // TODO: Return calculated g cost
    double g_cost = 0;
    int dx = abs(from->cell.x - goal->cell.x);
    int dy = abs(from->cell.y - goal->cell.y);
    if (dx + dy > 1){
        g_cost = SQ_2;
    } else{
        g_cost = 1;
    }
    return g_cost;
}

std::vector<Node*> expand_node(Node* node, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    // TODO: Return children of a given node that are not obstacles
    std::vector<Node*> children;
    // Expand to 8 ways
    const int x_deltas[8] = {1, -1, 0, 0, 1, -1, 1, -1};
    const int y_deltas[8] = {0, 0, 1, -1, 1, -1, -1, 1};
    int adjacent_cell_x;
    int adjacent_cell_y;

    for (int n = 0; n < 8; ++n)
    {
        adjacent_cell_x = node->cell.x + x_deltas[n];
        adjacent_cell_y = node->cell.y + y_deltas[n];

        if (distances.isCellInGrid(adjacent_cell_x, adjacent_cell_y))
        {
            float cell_distance = distances(adjacent_cell_x, adjacent_cell_y);
            if (cell_distance > params.minDistanceToObstacle) // && cell_distance<params.maxDistanceWithCost)
            {
                Node *adjacent_node = new Node(adjacent_cell_x, adjacent_cell_y);
                children.push_back(adjacent_node);
            }
        }
    }
    return children;
}

std::vector<Node*> extract_node_path(Node* goal_node, Node* start_node)
{
    // TODO: Generate path by following parent nodes
    std::vector<Node*> path;
    Node *cur_node = goal_node;
    while (cur_node->parent != start_node){
        path.push_back(cur_node);
        cur_node = cur_node->parent;
    }
    path.push_back(start_node);
    return path;
}

// To prune the path for the waypoint follower
std::vector<mbot_lcm_msgs::pose_xyt_t> extract_pose_path(std::vector<Node*> nodes, const ObstacleDistanceGrid& distances)
{
    // TODO: prune the path to generate sparse waypoints
    std::vector<mbot_lcm_msgs::pose_xyt_t> path;
    for (int i=nodes.size()-1; i>=0; i--){
        mbot_lcm_msgs::pose_xyt_t pose = {0};
        Point<double> global_pose = grid_position_to_global_position(nodes[i]->cell, distances);
        pose.x = global_pose.x;
        pose.y = global_pose.y;
        path.push_back(pose);
    }
    return path;
}

bool is_in_list(Node* node, std::vector<Node*> list)
{
    for (auto &&item : list)
    {
        if (*node == *item) return true;
    }
    return false;
}

Node* get_from_list(Node* node, std::vector<Node*> list)
{
    for (auto &&n : list)
    {
        if (*node == *n) return n;
    }
    return NULL;
    
}
