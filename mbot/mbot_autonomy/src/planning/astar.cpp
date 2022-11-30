#include <common_utils/timestamp.h>
#include <planning/astar.hpp>
#include <algorithm>
#include <chrono>

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
        if (*cur_node == goal_node){
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

    if (!path_found){
        path.path.clear();
        path.path.push_back(start);
        path.path_length = path.path.size();
        path.utime = utime_now();
        return path;
    }

    path.path_length = path.path.size();
    path.utime = utime_now();
    return path;
}



double h_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances)
{
    // TODO: Return calculated h cost
    double h_cost = 0;
    int dx = abs(from->cell.x - goal->cell.x);
    int dy = abs(from->cell.y - goal->cell.y);
    
    h_cost = (dx + dy) + (1.414 - 2) * std::min(dx, dy);
    return h_cost;
}
double g_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    // TODO: Return calculated g cost
    double g_cost = 0;
    int dx = abs(from->cell.x - goal->cell.x);
    int dy = abs(from->cell.y - goal->cell.y);
    if (dx + dy > 1){
        g_cost = 1.414;
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

    for (int n = 0; n < 8; ++n)
    {
        Node *adjacent_node = new Node(node->cell.x + x_deltas[n], node->cell.y + y_deltas[n]);
        if (distances.isCellInGrid(adjacent_node->cell.x, adjacent_node->cell.y))
        {
            // children.push_back(adjacent_node);
            float cell_distance = distances(adjacent_node->cell.x, adjacent_node->cell.y);
            if (cell_distance>params.minDistanceToObstacle){//} && cell_distance<params.maxDistanceWithCost){
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
