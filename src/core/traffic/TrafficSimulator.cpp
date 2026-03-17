#include "core/traffic/TrafficSimulator.h"
#include <algorithm>
#include <iostream>

namespace nav {

TrafficSimulator::TrafficSimulator(Graph& graph, PathFinder* pathfinder, unsigned int seed)
    : graph_(graph)
    , pathfinder_(pathfinder)
    , rng_(seed)
{
    // Cache all node IDs for random origin/destination selection
    for (const auto& nodePair : graph_.getNodes()) {
        allNodeIds_.push_back(nodePair.first);
    }
}

void TrafficSimulator::step() {
    changedEdges_.clear();
    stepCount_++;

    // Phase 1: Advance existing cars
    advanceCars();

    // Phase 2: Spawn new cars
    spawnCars();
}

void TrafficSimulator::advanceCars() {
    std::vector<Car> surviving;
    surviving.reserve(activeCars_.size());

    for (auto& car : activeCars_) {
        car.remainingTime -= timeStep_;

        if (car.remainingTime <= 0.0) {
            // Car finished traversing current edge
            Edge* currentEdge = graph_.getEdge(car.currentEdge);
            if (currentEdge) {
                currentEdge->decrementCarCount();
                changedEdges_.push_back(car.currentEdge);
            }

            // Move to next edge in route
            car.routeIndex++;
            if (car.routeIndex < car.route.size()) {
                Edge::Id nextEdgeId = car.route[car.routeIndex];
                Edge* nextEdge = graph_.getEdge(nextEdgeId);
                if (nextEdge) {
                    nextEdge->incrementCarCount();
                    car.currentEdge = nextEdgeId;
                    car.remainingTime = computeTravelTime(nextEdge);
                    changedEdges_.push_back(nextEdgeId);
                    surviving.push_back(car);
                }
                // If edge is invalid, car is removed (not added to surviving)
            }
            // If route is complete, car is removed (not added to surviving)
        } else {
            // Car still traveling on current edge
            surviving.push_back(car);
        }
    }

    activeCars_ = std::move(surviving);
}

void TrafficSimulator::spawnCars() {
    if (allNodeIds_.size() < 2 || !pathfinder_) {
        return;
    }

    // Limit active cars
    int toSpawn = spawnRate_;
    if (static_cast<int>(activeCars_.size()) + toSpawn > maxCars_) {
        toSpawn = maxCars_ - static_cast<int>(activeCars_.size());
    }
    if (toSpawn <= 0) {
        return;
    }

    std::uniform_int_distribution<size_t> nodeDist(0, allNodeIds_.size() - 1);

    for (int i = 0; i < toSpawn; ++i) {
        // Pick random origin and destination
        Node::Id origin = allNodeIds_[nodeDist(rng_)];
        Node::Id destination = allNodeIds_[nodeDist(rng_)];

        // Skip if same node
        if (origin == destination) {
            continue;
        }

        // Compute route
        PathResult result = pathfinder_->findPath(graph_, origin, destination);
        if (!result.found || result.pathEdges.empty()) {
            continue;
        }

        // Create car and place on first edge
        Car car;
        car.route = result.pathEdges;
        car.routeIndex = 0;
        car.currentEdge = car.route[0];

        Edge* firstEdge = graph_.getEdge(car.currentEdge);
        if (firstEdge) {
            firstEdge->incrementCarCount();
            car.remainingTime = computeTravelTime(firstEdge);
            changedEdges_.push_back(car.currentEdge);
            activeCars_.push_back(car);
        }
    }
}

double TrafficSimulator::computeTravelTime(const Edge* edge) const {
    if (!edge) return 1.0;
    return TrafficModel::calculateTravelTime(
        edge->getLength(),
        edge->getCapacity(),
        static_cast<double>(edge->getCarCount())
    );
}

void TrafficSimulator::reset() {
    // Remove all cars from edges
    for (auto& car : activeCars_) {
        Edge* edge = graph_.getEdge(car.currentEdge);
        if (edge) {
            edge->decrementCarCount();
        }
    }
    activeCars_.clear();
    changedEdges_.clear();
    stepCount_ = 0;

    // Reset all edge car counts to zero
    for (const auto& edgePair : graph_.getEdges()) {
        Edge* edge = graph_.getEdge(edgePair.first);
        if (edge) {
            edge->setCarCount(0);
        }
    }
}

} // namespace nav
