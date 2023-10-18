#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x, start_y);    // two different ways to get same result :EIH.
    end_node   = &m_Model.FindClosestNode(end_x, end_y);
   
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
   // return std::abs(node->x - end_node->x) + std::abs(node->y - end_node->y);  // Method used in the class excersize 
    return std::sqrt(std::pow((node->x - end_node->x), 2) + std::pow((node->y - end_node->y), 2));
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:     
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    
    current_node->FindNeighbors(); 
    current_node->visited = true;             // Adding this mark first node as visited useful for init start_note in A*Search method;
                                              // Following straighforward intructions above 
    for (auto index_node : current_node->neighbors){
        index_node->visited = true;
        index_node->parent = current_node;
        index_node->h_value = CalculateHValue(index_node);
        index_node->g_value = current_node->g_value + current_node->distance(*index_node);   
        open_list.emplace_back(index_node);   
    }
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
             // Compare method using Lamda expresion inline function approach 
             // [](RouteModel::Node* a, RouteModel::Node* b){ return (a->g_value + a->h_value) < (b->g_value + b->h_value)

    std::sort(open_list.begin(), open_list.end(), [](RouteModel::Node* a, RouteModel::Node* b) 
                                                   {return (a->g_value + a->h_value) < (b->g_value + b->h_value);});    
    auto pointer_lowest = *(open_list.begin());     // assigning the address of the lowest node  
    open_list.erase(open_list.begin());             // Removing from the open list.

   return pointer_lowest;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    // TODO: Implement your solution here.
    
    while(current_node->parent != nullptr){  
        
        distance += current_node->distance(*(current_node->parent)); // Calculating the cost between current node and parent's node
        path_found.emplace_back(*current_node);          // saving the current node in the path_found vector
        current_node = current_node->parent;             // Parent node is now the current node 
    }   

    path_found.emplace_back(*current_node);             // Saving the last node "start_node" is the path_found vector
    reverse(path_found.begin(), path_found.end());      // Flipping the order in path_found vector
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
   // TODO: Implement your solution here.
    AddNeighbors(start_node);     // Init Open.list with Start Node values
        
    while (open_list.size() > 0) {                                                  // compare is not empty
        
        current_node = NextNode();                                                  // assing next nearest node to current_node

        if((current_node->x == end_node->x) && (current_node->y == end_node->y)){   // Checking If End_goal is reached 
           m_Model.path = ConstructFinalPath(current_node);                         // Path_found assigned  
           return;
        }
        AddNeighbors(current_node);                                                // Loopin to next nearest neightbords nodes
    
    } 
}  
