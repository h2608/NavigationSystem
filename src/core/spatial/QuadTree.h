#ifndef QUADTREE_H
#define QUADTREE_H

#include "core/spatial/BoundingBox.h"
#include "core/graph/Node.h"
#include <vector>
#include <memory>
#include <queue>

namespace nav {

// QuadTree for efficient spatial indexing of nodes
class QuadTree {
public:
    static constexpr size_t MAX_CAPACITY = 4;

    // Constructor
    explicit QuadTree(const BoundingBox& bounds);

    // Insert a node into the tree
    bool insert(Node::Id nodeId, const Point2D& position);

    // Find k nearest neighbors to a query point
    std::vector<Node::Id> findKNearest(const Point2D& query, size_t k) const;

    // Query all nodes within a bounding box
    std::vector<Node::Id> queryRange(const BoundingBox& range) const;

    // Clear the tree
    void clear();

private:
    struct NodeEntry {
        Node::Id id;
        Point2D position;

        NodeEntry(Node::Id nodeId, const Point2D& pos)
            : id(nodeId), position(pos) {}
    };

    // Helper struct for k-nearest search (max heap)
    struct DistanceEntry {
        Node::Id id;
        double distance;

        DistanceEntry(Node::Id nodeId, double dist)
            : id(nodeId), distance(dist) {}

        // For max heap (priority_queue uses less by default, so we reverse)
        bool operator<(const DistanceEntry& other) const {
            return distance < other.distance;
        }
    };

    // Subdivide this node into 4 children
    void subdivide();

    // Helper for k-nearest search using max-heap
    void findKNearestHelper(const Point2D& query, size_t k,
                           std::priority_queue<DistanceEntry>& maxHeap) const;

    // Helper for range query
    void queryRangeHelper(const BoundingBox& range, std::vector<Node::Id>& result) const;

    BoundingBox bounds_;
    std::vector<NodeEntry> entries_;
    bool divided_;

    // Children (NW, NE, SW, SE)
    std::unique_ptr<QuadTree> northWest_;
    std::unique_ptr<QuadTree> northEast_;
    std::unique_ptr<QuadTree> southWest_;
    std::unique_ptr<QuadTree> southEast_;
};

} // namespace nav

#endif // QUADTREE_H
