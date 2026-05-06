#include "core/graph/Edge.h"

namespace nav {

Edge::Edge()
    : id_(INVALID_ID)
    , source_(Node::INVALID_ID)
    , target_(Node::INVALID_ID)
    , length_(0.0)
    , capacity_(10.0)
    , carCount_(0)
    , roadClass_(RoadClass::Local)
    , importanceScore_(defaultImportanceForRoadClass(RoadClass::Local))
    , displayTier_(defaultDisplayTierForRoadClass(RoadClass::Local))
    , oneWay_(false)
{}

Edge::Edge(Id id, Node::Id source, Node::Id target, double length)
    : id_(id)
    , source_(source)
    , target_(target)
    , length_(length)
    , capacity_(10.0)
    , carCount_(0)
    , roadClass_(RoadClass::Local)
    , importanceScore_(defaultImportanceForRoadClass(RoadClass::Local))
    , displayTier_(defaultDisplayTierForRoadClass(RoadClass::Local))
    , oneWay_(false)
{}

} // namespace nav
