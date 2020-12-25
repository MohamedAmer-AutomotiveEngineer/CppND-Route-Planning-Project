#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node   = &m_Model.FindClosestNode(end_x, end_y);
}


// Implementing the CalculateHValue method.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  
    return node->distance(*(this->end_node));
}


// Completing the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for(auto neighborNode : current_node->neighbors)
    {
      neighborNode->parent = current_node;
      neighborNode->h_value = CalculateHValue(neighborNode);
      neighborNode->g_value = current_node->g_value + neighborNode->distance(*(current_node));
      neighborNode->visited = true;
      open_list.push_back(neighborNode);
    }
}


// Implementing the NextNode method to sort the open list and return the next node.
RouteModel::Node *RoutePlanner::NextNode() {
    sort(open_list.begin(), open_list.end(), [] (const RouteModel::Node* a, const RouteModel::Node* b) {return a->g_value + a->h_value > b->g_value + b->h_value;});
    auto current = open_list.back();
    open_list.pop_back();
    return current;
}

// Implementing the ConstructFinalPath method to return the final path found from your A* search.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    RouteModel::Node *node = current_node;
    while(node != start_node)
    {
      distance += node->distance(*(node->parent));
      path_found.emplace_back(*node);
      node = node->parent;
    }
    path_found.emplace_back(*node);
    
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// Implementing the A* Search algorithm.
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
    start_node->visited = true; // This line has been inherited from patrickjmcgoldrick/CppND-Route-Planning-Project/blob/master/src/route_planner.cpp repo.
    open_list.push_back(start_node);
    while(open_list.size() > 0)
    {
      current_node = NextNode();
      if(current_node == end_node)
      {
        m_Model.path = ConstructFinalPath(current_node);
      }
      else
      {
        AddNeighbors(current_node);
      }
    }
}