#ifndef TRAFFICSIMULATOR_H
#define TRAFFICSIMULATOR_H

#include "core/algorithms/PathFinder.h"
#include "core/graph/Graph.h"
#include "core/traffic/TrafficModel.h"

#include <random>
#include <unordered_set>
#include <vector>

namespace nav {

class TrafficSimulator {
public:
    struct Car {
        Edge::Id currentEdge = Edge::INVALID_ID;
        Node::Id currentNode = Node::INVALID_ID;
        Node::Id destination = Node::INVALID_ID;
        double remainingTime = 0.0;
        std::vector<Edge::Id> route;
        size_t routeIndex = 0;
        int rerouteCooldownEdges = 0;
    };

    TrafficSimulator(Graph& graph, PathFinder* pathfinder, unsigned int seed = 42);

    void step();

    const std::vector<Edge::Id>& getChangedEdges() const { return changedEdges_; }
    Graph& getGraph() { return graph_; }
    size_t getActiveCarCount() const { return activeCars_.size(); }

    void reset();

    void setSpawnRate(int carsPerStep) { spawnRate_ = carsPerStep; }
    void setMaxCars(int maxCars) { maxCars_ = maxCars; }
    void setTimeStep(double dt) { timeStep_ = dt; }

private:
    void cacheDemandNodes();
    void spawnCars();
    void advanceCars(int& remainingReroutes);
    bool moveToNextEdge(Car& car, int& remainingReroutes);
    bool shouldReroute(const Car& car, Edge::Id nextEdgeId) const;
    bool rerouteFromCurrentNode(Car& car);
    bool enterEdge(Car& car, Edge::Id edgeId, size_t routeIndex);
    Node::Id chooseDemandNode();
    double computeTravelTime(const Edge* edge) const;
    void markEdgeChanged(Edge::Id id);

    Graph& graph_;
    PathFinder* pathfinder_;
    std::mt19937 rng_;

    std::vector<Car> activeCars_;
    std::vector<Edge::Id> changedEdges_;
    std::unordered_set<Edge::Id> changedEdgeSet_;
    std::vector<Node::Id> demandNodeIds_;
    std::vector<double> demandNodeWeights_;

    int spawnRate_ = 10;
    int maxCars_ = 1000;
    double timeStep_ = 1.0;
    int stepCount_ = 0;
};

} // namespace nav

#endif // TRAFFICSIMULATOR_H
