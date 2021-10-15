//
// Created by caochao on 7/12/19.
//

#include "../../include/tsp_solver/tsp_solver.h"

namespace tsp_solver_ns
{
TSPSolver::TSPSolver(tsp_solver_ns::DataModel data) : data_(std::move(data))
{
  // Create Routing Index Manager
  // Number of rows/locations, depot node which is the start and end location
  manager_ = std::make_unique<RoutingIndexManager>(data_.distance_matrix.size(), data_.num_vehicles, data_.depot);

  // Create Routing Model.
  routing_ = std::make_unique<RoutingModel>(*manager_);
}

/**
 * Sets up the solver, solves the problem and stores in solution_. Defines the distance callback which returns the dist between 2 locations
 */
void TSPSolver::Solve()
{
  // first line registers the distance callback with the solver (solver's internal reference) as transit_callback_index. Argument is a lambda function which defines the callback. 
  const int transit_callback_index =
      routing_->RegisterTransitCallback([this](int64 from_index, int64 to_index) -> int64 {
        // Convert from routing variable Index to distance matrix NodeIndex.
        // IndexToNode converts internal indices to the location index
        auto from_node = manager_->IndexToNode(from_index).value();
        auto to_node = manager_->IndexToNode(to_index).value();
        return data_.distance_matrix[from_node][to_node];
      });

  // Define cost of each arc.
  // the arc cost evaluator tells the solver how to calculate the cost of each joining edge in the graph. In this case, using the callback defined above.
  routing_->SetArcCostEvaluatorOfAllVehicles(transit_callback_index);

  // Setting first solution heuristic.
  // PATH_CHEAPEST_ARC creates initial route for the solver by adding edges wit hteh least weight that don't lead to a previously visited node
  RoutingSearchParameters searchParameters = DefaultRoutingSearchParameters();
  searchParameters.set_first_solution_strategy(FirstSolutionStrategy::PATH_CHEAPEST_ARC);

  // Solve the problem.
  solution_ = routing_->SolveWithParameters(searchParameters);
}

/**
 * Extracts the route from solution_. Print the route and total route distance
 * TODO wall_time() is time elapsed since creation of solver, which is not computation time taken? 
 */
void TSPSolver::PrintSolution()
{
  // Inspect solution.
  LOG(INFO) << "Objective: " << (solution_->ObjectiveValue()) / 10.0 << " meters";
  int64 index = routing_->Start(0);
  LOG(INFO) << "Route:";
  int64 distance{ 0 };
  std::stringstream route;
  while (routing_->IsEnd(index) == false)
  {
    route << manager_->IndexToNode(index).value() << " -> ";
    int64 previous_index = index;
    index = solution_->Value(routing_->NextVar(index));
    distance += const_cast<RoutingModel&>(*routing_).GetArcCostForVehicle(previous_index, index, 0LL);
  }
  LOG(INFO) << route.str() << manager_->IndexToNode(index).value();
  LOG(INFO) << "Route distance: " << distance / 10.0 << " meters";
  LOG(INFO) << "Problem solved in " << routing_->solver()->wall_time() << "ms";
}
/**
 * TODO wall_time() is time elapsed since creation of solver, which is not computation time taken? 
 */
int TSPSolver::getComputationTime()
{
  return routing_->solver()->wall_time();
}
/**
 * Stores nodes of route in vector node_index. Removes the dummy node if there is one. Dummy node has 0 dist to start node, but 9999 to every other nodes (see local_coverage_planner.cpp)
 * @param has_dummy bool indicating if there is an additional dummy node
 */
void TSPSolver::getSolutionNodeIndex(std::vector<int>& node_index, bool has_dummy)
{
  node_index.clear();
  int index = routing_->Start(0);
  int end_index = index;
  while (routing_->IsEnd(index) == false)
  {
    node_index.push_back(manager_->IndexToNode(index).value());
    index = solution_->Value(routing_->NextVar(index));
  }
  // push back the end node index
  //       node_index.push_back(end_index);
  if (has_dummy)
  {
    int dummy_node_index = data_.distance_matrix.size() - 1;
    if (node_index[1] == dummy_node_index) // dummy node is the second entry, after the start node.
    {
      // delete dummy node
      node_index.erase(node_index.begin() + 1); // erase() takes in an iterator such as .begin() which points to first element
      // push the start node to the end
      node_index.push_back(node_index[0]);
      // remove the start node at the begining
      node_index.erase(node_index.begin());
      // reverse the whole array
      std::reverse(node_index.begin(), node_index.end());
    }
    else  // the last node is dummy node
    {
      node_index.pop_back();
    }
  }
}

double TSPSolver::getPathLength()
{
  return (solution_->ObjectiveValue()) / 10.0;
}

}  // namespace tsp_solver_ns
