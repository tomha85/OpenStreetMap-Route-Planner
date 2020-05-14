#include<algorithm>
#include"route_planner.h"

RoutePlanner::RoutePlanner(RouteModel &model,float start_x,float start_y,float end_x,float end_y):m_Model(model)
{
  start_x*=0.01;
  start_y*=0.01;
  end_x*=0.01;
  end_y*=0.01;
  start_node=&model.FindClosestNode(start_x,start_y);
  end_node=&model.FindClosestNode(end_x,end_y);
}
float RoutePlanner::CalculateHValue(RouteModel::Node const *node)
{
  float k;
  k=node->distance(*end_node);
  return k;
}
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node)
{
  current_node->FindNeighbors();
  for(auto neighbor:current_node->neighbors)
  {
    neighbor->parent=current_node;
    neighbor->g_value=current_node->g_value + current_node->distance(*neighbor);
    neighbor->h_value = CalculateHValue(neighbor);
    neighbor->visited=true;
    open_list.push_back(neighbor);
  }
}

bool compareVl(const RouteModel::Node *a,const RouteModel::Node *b)
{
  float f1=(a->g_value+a->h_value);
  float f2=(b->g_value+b->h_value);
  return f1>f2;
}

RouteModel::Node *RoutePlanner::NextNode()
{
  std::sort(open_list.begin(),open_list.end(),compareVl);
  RouteModel::Node *NearNode=open_list.back();
  open_list.pop_back();
  return NearNode;
}
std::vector<RouteModel::Node>RoutePlanner::ConstructFinalPath(RouteModel::Node*current_node)
{
  std::vector<RouteModel::Node>path_found;
  while(current_node->x!=start_node->x && current_node->y!=start_node->y)
  {
    distance +=current_node->distance(*(current_node->parent));
    path_found.push_back(*current_node);
    current_node=current_node->parent;    
  }
  path_found.push_back(*current_node);
  distance*=m_Model.MetricScale();
  std::reverse(path_found.begin(),path_found.end());

  return path_found;

}
void RoutePlanner::AStarSearch() {
  RouteModel::Node *current_node = nullptr;
  start_node->visited = true;
  open_list.push_back(start_node);

  while (!open_list.empty()) {

    current_node = NextNode();

    if (current_node->distance(*end_node) == 0) 
    {
      m_Model.path = ConstructFinalPath(current_node);
      return;
    } 
    else 
    {
      AddNeighbors(current_node);
    }
  } 
}
