#ifndef EDGE_H
#define EDGE_H

#include "core/graph/Node.h"
#include <cstdint>
#include <string>

namespace nav {

enum class RoadClass : uint8_t {
    Arterial  = 0,
    Secondary = 1,
    Local     = 2
};

enum class DisplayTier : uint8_t {
    Primary   = 0,
    Secondary = 1,
    Local     = 2,
    Detail    = 3
};

inline double roadClassSpeedFactor(RoadClass rc) {
    switch (rc) {
        case RoadClass::Arterial:  return 2.0;
        case RoadClass::Secondary: return 1.5;
        case RoadClass::Local:
        default:                   return 1.0;
    }
}

inline double defaultImportanceForRoadClass(RoadClass rc) {
    switch (rc) {
        case RoadClass::Arterial:  return 0.90;
        case RoadClass::Secondary: return 0.62;
        case RoadClass::Local:
        default:                   return 0.24;
    }
}

inline DisplayTier defaultDisplayTierForRoadClass(RoadClass rc) {
    switch (rc) {
        case RoadClass::Arterial:  return DisplayTier::Primary;
        case RoadClass::Secondary: return DisplayTier::Secondary;
        case RoadClass::Local:
        default:                   return DisplayTier::Local;
    }
}

inline DisplayTier displayTierForImportance(double importance) {
    if (importance >= 0.82) return DisplayTier::Primary;
    if (importance >= 0.58) return DisplayTier::Secondary;
    if (importance >= 0.32) return DisplayTier::Local;
    return DisplayTier::Detail;
}

class Edge {
public:
    using Id = uint32_t;
    static constexpr Id INVALID_ID = UINT32_MAX;

    Edge();
    Edge(Id id, Node::Id source, Node::Id target, double length);

    Id getId() const { return id_; }
    Node::Id getSource() const { return source_; }
    Node::Id getTarget() const { return target_; }
    double getLength() const { return length_; }
    double getCapacity() const { return capacity_; }
    uint32_t getCarCount() const { return carCount_; }
    RoadClass getRoadClass() const { return roadClass_; }
    double getImportanceScore() const { return importanceScore_; }
    DisplayTier getDisplayTier() const { return displayTier_; }
    bool isOneWay() const { return oneWay_; }
    const std::string& getRoadName() const { return roadName_; }
    Node::Id getTraversableTarget(Node::Id nodeId) const {
        if (nodeId == source_) return target_;
        if (!oneWay_ && nodeId == target_) return source_;
        return Node::INVALID_ID;
    }
    Node::Id getOppositeNode(Node::Id nodeId) const {
        if (nodeId == source_) return target_;
        if (nodeId == target_) return source_;
        return Node::INVALID_ID;
    }

    void setCapacity(double capacity) { capacity_ = capacity; }
    void setCarCount(uint32_t count) { carCount_ = count; }
    void setRoadClass(RoadClass rc) { roadClass_ = rc; }
    void setImportanceScore(double importance) { importanceScore_ = importance; }
    void setDisplayTier(DisplayTier tier) { displayTier_ = tier; }
    void setOneWay(bool oneWay) { oneWay_ = oneWay; }
    void setRoadName(const std::string& roadName) { roadName_ = roadName; }

    void incrementCarCount() { ++carCount_; }
    void decrementCarCount() { if (carCount_ > 0) --carCount_; }

private:
    Id id_;
    Node::Id source_;
    Node::Id target_;
    double length_;
    double capacity_;
    uint32_t carCount_;
    RoadClass roadClass_ = RoadClass::Local;
    double importanceScore_ = defaultImportanceForRoadClass(RoadClass::Local);
    DisplayTier displayTier_ = defaultDisplayTierForRoadClass(RoadClass::Local);
    bool oneWay_ = false;
    std::string roadName_;
};

} // namespace nav

#endif // EDGE_H
