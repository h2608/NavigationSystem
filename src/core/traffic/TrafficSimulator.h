#ifndef TRAFFICSIMULATOR_H
#define TRAFFICSIMULATOR_H

#include "core/graph/Graph.h"
#include "core/traffic/TrafficModel.h"
#include "core/algorithms/PathFinder.h"
#include <random>
#include <vector>

namespace nav {

class TrafficSimulator {
public:
    // Car agent that travels along a computed route
    struct Car {
        Edge::Id currentEdge;       // Road this car is currently on
        double remainingTime;       // Time left before exiting this road
        std::vector<Edge::Id> route; // Planned route (sequence of edges)
        size_t routeIndex;          // Current position in route
    };

    TrafficSimulator(Graph& graph, PathFinder* pathfinder, unsigned int seed = 42);

    // Perform one simulation step (advance all cars, spawn new ones)
    void step();

    // Get list of edges that changed in last step
    const std::vector<Edge::Id>& getChangedEdges() const { return changedEdges_; }

    // Get graph reference for visual updates
    Graph& getGraph() { return graph_; }

    // Get number of active cars
    size_t getActiveCarCount() const { return activeCars_.size(); }

    // Reset all traffic to zero
    void reset();

    // Configuration
    void setSpawnRate(int carsPerStep) { spawnRate_ = carsPerStep; }
    void setMaxCars(int maxCars) { maxCars_ = maxCars; }
    void setTimeStep(double dt) { timeStep_ = dt; }

private:
    void spawnCars();
    void advanceCars();
    double computeTravelTime(const Edge* edge) const;

    Graph& graph_;
    PathFinder* pathfinder_;
    std::mt19937 rng_;

    std::vector<Car> activeCars_;
    std::vector<Edge::Id> changedEdges_;
    std::vector<Node::Id> allNodeIds_;

    // Simulation parameters
    int spawnRate_ = 10;     // Cars to spawn per step
    int maxCars_ = 1000;     // Maximum active cars
    double timeStep_ = 1.0;  // Time units per step
    int stepCount_ = 0;
};

} // namespace nav

#endif // TRAFFICSIMULATOR_H
