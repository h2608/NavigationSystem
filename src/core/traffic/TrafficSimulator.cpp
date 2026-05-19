#include "core/traffic/TrafficSimulator.h"

#include <algorithm>
#include <utility>

namespace nav {

namespace {

constexpr double kDemandWeightFloor = 0.1;
constexpr int kSpawnRouteAttempts = 4;
constexpr int kRerouteCooldownEdges = 3;
constexpr int kMinReroutesPerStep = 8;
constexpr int kMaxReroutesPerStep = 64;

} // namespace

TrafficSimulator::TrafficSimulator(Graph& graph, PathFinder* pathfinder, unsigned int seed)
    : graph_(graph)
    , pathfinder_(pathfinder)
    , rng_(seed)
{
    cacheDemandNodes();
}

void TrafficSimulator::step() {
    changedEdges_.clear();
    changedEdgeSet_.clear();
    stepCount_++;

    int remainingReroutes = std::clamp(spawnRate_ * 2, kMinReroutesPerStep, kMaxReroutesPerStep);
    advanceCars(remainingReroutes);
    spawnCars();
}

void TrafficSimulator::cacheDemandNodes() {
    demandNodeIds_.clear();
    demandNodeWeights_.clear();

    for (const auto& nodePair : graph_.getNodes()) {
        const Node::Id nodeId = nodePair.first;
        double weight = 0.0;

        for (Edge::Id edgeId : graph_.getAdjacentEdges(nodeId)) {
            const Edge* edge = graph_.getEdge(edgeId);
            if (!edge || edge->getTraversableTarget(nodeId) == Node::INVALID_ID) {
                continue;
            }

            const double capacity = std::max(edge->getCapacity(), kDemandWeightFloor);
            weight += capacity * roadClassSpeedFactor(edge->getRoadClass());
        }

        if (weight > 0.0) {
            demandNodeIds_.push_back(nodeId);
            demandNodeWeights_.push_back(weight);
        }
    }
}

void TrafficSimulator::advanceCars(int& remainingReroutes) {
    size_t writeIndex = 0;

    for (size_t readIndex = 0; readIndex < activeCars_.size(); ++readIndex) {
        Car car = std::move(activeCars_[readIndex]);
        car.remainingTime -= timeStep_;

        if (car.remainingTime > 0.0) {
            activeCars_[writeIndex++] = std::move(car);
            continue;
        }

        Edge* currentEdge = graph_.getEdge(car.currentEdge);
        if (!currentEdge) {
            continue;
        }

        const Node::Id arrivalNode = currentEdge->getTraversableTarget(car.currentNode);
        currentEdge->decrementCarCount();
        markEdgeChanged(car.currentEdge);

        if (arrivalNode == Node::INVALID_ID) {
            continue;
        }

        car.currentNode = arrivalNode;
        if (car.currentNode == car.destination) {
            continue;
        }

        if (car.rerouteCooldownEdges > 0) {
            --car.rerouteCooldownEdges;
        }

        if (moveToNextEdge(car, remainingReroutes)) {
            activeCars_[writeIndex++] = std::move(car);
        }
    }

    activeCars_.resize(writeIndex);
}

bool TrafficSimulator::moveToNextEdge(Car& car, int& remainingReroutes) {
    size_t nextRouteIndex = car.routeIndex + 1;
    Edge::Id nextEdgeId = Edge::INVALID_ID;
    bool routeValid = false;

    if (nextRouteIndex < car.route.size()) {
        nextEdgeId = car.route[nextRouteIndex];
        const Edge* nextEdge = graph_.getEdge(nextEdgeId);
        routeValid = nextEdge && nextEdge->getTraversableTarget(car.currentNode) != Node::INVALID_ID;
    }

    const bool routeNeedsRepair = !routeValid;
    const bool routeShouldAvoid = routeValid && shouldReroute(car, nextEdgeId);
    if ((routeNeedsRepair || routeShouldAvoid) &&
        remainingReroutes > 0 &&
        (routeNeedsRepair || car.rerouteCooldownEdges == 0)) {
        --remainingReroutes;
        if (rerouteFromCurrentNode(car)) {
            nextRouteIndex = 0;
            nextEdgeId = car.route.front();
            routeValid = true;
        } else if (routeNeedsRepair) {
            return false;
        }
    }

    if (!routeValid) {
        return false;
    }

    return enterEdge(car, nextEdgeId, nextRouteIndex);
}

bool TrafficSimulator::shouldReroute(const Car& car, Edge::Id nextEdgeId) const {
    if (car.rerouteCooldownEdges > 0) {
        return false;
    }

    const Edge* edge = graph_.getEdge(nextEdgeId);
    if (!edge) {
        return true;
    }

    return TrafficModel::getCongestionRatio(
        edge->getCapacity(),
        static_cast<double>(edge->getCarCount())
    ) >= TrafficModel::VISUAL_RED_RATIO;
}

bool TrafficSimulator::rerouteFromCurrentNode(Car& car) {
    if (!pathfinder_ ||
        car.currentNode == Node::INVALID_ID ||
        car.destination == Node::INVALID_ID ||
        car.currentNode == car.destination) {
        return false;
    }

    PathResult result = pathfinder_->findPath(graph_, car.currentNode, car.destination);
    if (!result.found || result.pathEdges.empty()) {
        return false;
    }

    car.route = std::move(result.pathEdges);
    car.routeIndex = 0;
    car.rerouteCooldownEdges = kRerouteCooldownEdges;
    return true;
}

void TrafficSimulator::spawnCars() {
    if (demandNodeIds_.size() < 2 || !pathfinder_) {
        return;
    }

    const int availableSlots = std::max(0, maxCars_ - static_cast<int>(activeCars_.size()));
    if (availableSlots <= 0) {
        return;
    }

    std::poisson_distribution<int> spawnDist(static_cast<double>(std::max(0, spawnRate_)));
    const int toSpawn = std::min(spawnDist(rng_), availableSlots);

    for (int i = 0; i < toSpawn; ++i) {
        for (int attempt = 0; attempt < kSpawnRouteAttempts; ++attempt) {
            const Node::Id origin = chooseDemandNode();
            const Node::Id destination = chooseDemandNode();
            if (origin == Node::INVALID_ID || destination == Node::INVALID_ID || origin == destination) {
                continue;
            }

            PathResult result = pathfinder_->findPath(graph_, origin, destination);
            if (!result.found || result.pathEdges.empty()) {
                continue;
            }

            Car car;
            car.currentNode = origin;
            car.destination = destination;
            car.route = std::move(result.pathEdges);

            if (!enterEdge(car, car.route.front(), 0)) {
                continue;
            }

            activeCars_.push_back(std::move(car));
            break;
        }
    }
}

bool TrafficSimulator::enterEdge(Car& car, Edge::Id edgeId, size_t routeIndex) {
    Edge* edge = graph_.getEdge(edgeId);
    if (!edge || edge->getTraversableTarget(car.currentNode) == Node::INVALID_ID) {
        return false;
    }

    edge->incrementCarCount();
    car.currentEdge = edgeId;
    car.routeIndex = routeIndex;
    car.remainingTime = computeTravelTime(edge);
    markEdgeChanged(edgeId);
    return true;
}

Node::Id TrafficSimulator::chooseDemandNode() {
    if (demandNodeIds_.empty()) {
        return Node::INVALID_ID;
    }

    std::discrete_distribution<size_t> nodeDist(demandNodeWeights_.begin(), demandNodeWeights_.end());
    return demandNodeIds_[nodeDist(rng_)];
}

double TrafficSimulator::computeTravelTime(const Edge* edge) const {
    if (!edge) {
        return 1.0;
    }

    const double baseTime = TrafficModel::calculateTravelTime(
        edge->getLength(),
        edge->getCapacity(),
        static_cast<double>(edge->getCarCount())
    );
    return baseTime / roadClassSpeedFactor(edge->getRoadClass());
}

void TrafficSimulator::markEdgeChanged(Edge::Id id) {
    if (id != Edge::INVALID_ID && changedEdgeSet_.insert(id).second) {
        changedEdges_.push_back(id);
    }
}

void TrafficSimulator::reset() {
    for (auto& car : activeCars_) {
        Edge* edge = graph_.getEdge(car.currentEdge);
        if (edge) {
            edge->decrementCarCount();
        }
    }

    activeCars_.clear();
    changedEdges_.clear();
    changedEdgeSet_.clear();
    stepCount_ = 0;

    for (const auto& edgePair : graph_.getEdges()) {
        Edge* edge = graph_.getEdge(edgePair.first);
        if (edge) {
            edge->setCarCount(0);
        }
    }
}

} // namespace nav
