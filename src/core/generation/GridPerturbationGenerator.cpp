#include "core/generation/GridPerturbationGenerator.h"
#include <cmath>
#include <iostream>
#include <algorithm>

namespace nav {

// ============================================================================
// Union-Find (Disjoint Set Union) for Kruskal's MST algorithm
// ============================================================================
namespace {

class UnionFind {
public:
    explicit UnionFind(size_t n) : parent_(n), rank_(n, 0) {
        for (size_t i = 0; i < n; ++i) {
            parent_[i] = i;
        }
    }

    size_t find(size_t x) {
        if (parent_[x] != x) {
            parent_[x] = find(parent_[x]);  // Path compression
        }
        return parent_[x];
    }

    bool unite(size_t x, size_t y) {
        size_t rootX = find(x);
        size_t rootY = find(y);
        if (rootX == rootY) {
            return false;  // Already in same set
        }
        // Union by rank
        if (rank_[rootX] < rank_[rootY]) {
            parent_[rootX] = rootY;
        } else if (rank_[rootX] > rank_[rootY]) {
            parent_[rootY] = rootX;
        } else {
            parent_[rootY] = rootX;
            rank_[rootX]++;
        }
        return true;
    }

private:
    std::vector<size_t> parent_;
    std::vector<size_t> rank_;
};

// Edge structure for MST calculation
struct TempEdge {
    Node::Id source;
    Node::Id target;
    double weight;  // Random weight for MST randomization
};

} // anonymous namespace

GridPerturbationGenerator::GridPerturbationGenerator()
    : rng_(std::random_device{}())
    , perturbationFactor_(0.4)
    , addDiagonals_(true)
    , diagonalProbability_(0.5)
{}

GridPerturbationGenerator::GridPerturbationGenerator(unsigned int seed)
    : rng_(seed)
    , perturbationFactor_(0.4)
    , addDiagonals_(true)
    , diagonalProbability_(0.5)
{}

void GridPerturbationGenerator::generate(Graph& graph, int numNodes, double width, double height) {
    // Clear existing graph
    graph.clear();

    if (numNodes <= 0) {
        return;
    }

    // Calculate grid dimensions (approximately square)
    int cols = static_cast<int>(std::ceil(std::sqrt(numNodes * width / height)));
    int rows = static_cast<int>(std::ceil(static_cast<double>(numNodes) / cols));

    // Adjust if we have too many cells
    while (rows * cols > numNodes * 1.5) {
        if (cols > rows) {
            cols--;
        } else {
            rows--;
        }
    }

    std::cout << "Generating " << numNodes << " nodes in a " << rows << "x" << cols
              << " grid (" << rows * cols << " cells)" << std::endl;

    // Calculate cell dimensions
    double cellWidth = width / cols;
    double cellHeight = height / rows;

    // Create distribution for perturbation
    std::uniform_real_distribution<double> perturbDist(-perturbationFactor_, perturbationFactor_);

    // Store node IDs in a 2D grid for easy neighbor lookup
    std::vector<std::vector<Node::Id>> nodeGrid(rows, std::vector<Node::Id>(cols, Node::INVALID_ID));

    // Generate nodes with perturbation
    int nodesCreated = 0;
    for (int i = 0; i < rows && nodesCreated < numNodes; ++i) {
        for (int j = 0; j < cols && nodesCreated < numNodes; ++j) {
            // Calculate base position (center of cell)
            double baseX = (j + 0.5) * cellWidth;
            double baseY = (i + 0.5) * cellHeight;

            // Add random perturbation
            double offsetX = perturbDist(rng_) * cellWidth;
            double offsetY = perturbDist(rng_) * cellHeight;

            // Clamp to stay within cell bounds (with small margin)
            double x = std::max(j * cellWidth + cellWidth * 0.05,
                              std::min(baseX + offsetX, (j + 1) * cellWidth - cellWidth * 0.05));
            double y = std::max(i * cellHeight + cellHeight * 0.05,
                              std::min(baseY + offsetY, (i + 1) * cellHeight - cellHeight * 0.05));

            // Create node
            Node::Id nodeId = graph.addNode(Point2D(x, y));
            nodeGrid[i][j] = nodeId;
            nodesCreated++;
        }
    }

    std::cout << "Created " << nodesCreated << " nodes" << std::endl;

    // ========================================================================
    // Step 1: Collect all potential edges (grid connectivity + diagonals)
    // ========================================================================
    std::vector<TempEdge> allEdges;
    std::uniform_real_distribution<double> weightDist(0.0, 1.0);
    std::uniform_real_distribution<double> diagChance(0.0, 1.0);

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            Node::Id currentNode = nodeGrid[i][j];
            if (currentNode == Node::INVALID_ID) {
                continue;
            }

            // Right neighbor (horizontal edge)
            if (j + 1 < cols && nodeGrid[i][j + 1] != Node::INVALID_ID) {
                allEdges.push_back({currentNode, nodeGrid[i][j + 1], weightDist(rng_)});
            }

            // Bottom neighbor (vertical edge)
            if (i + 1 < rows && nodeGrid[i + 1][j] != Node::INVALID_ID) {
                allEdges.push_back({currentNode, nodeGrid[i + 1][j], weightDist(rng_)});
            }

            // Diagonal edges (checkerboard pattern prevents X-crossings)
            if (addDiagonals_ && (i + j) % 2 == 0) {
                // Bottom-right diagonal
                if (i + 1 < rows && j + 1 < cols && nodeGrid[i + 1][j + 1] != Node::INVALID_ID) {
                    if (diagChance(rng_) < diagonalProbability_) {
                        allEdges.push_back({currentNode, nodeGrid[i + 1][j + 1], weightDist(rng_)});
                    }
                }

                // Bottom-left diagonal
                if (i + 1 < rows && j - 1 >= 0 && nodeGrid[i + 1][j - 1] != Node::INVALID_ID) {
                    if (diagChance(rng_) < diagonalProbability_) {
                        allEdges.push_back({currentNode, nodeGrid[i + 1][j - 1], weightDist(rng_)});
                    }
                }
            }
        }
    }

    std::cout << "Collected " << allEdges.size() << " potential edges" << std::endl;

    // ========================================================================
    // Step 2: Run Kruskal's Algorithm to find MST
    // ========================================================================
    // Sort edges by random weight
    std::sort(allEdges.begin(), allEdges.end(),
              [](const TempEdge& a, const TempEdge& b) { return a.weight < b.weight; });

    UnionFind uf(static_cast<size_t>(nodesCreated));
    std::vector<TempEdge> mstEdges;
    std::vector<TempEdge> nonMstEdges;

    for (const auto& edge : allEdges) {
        if (uf.unite(edge.source, edge.target)) {
            mstEdges.push_back(edge);
        } else {
            nonMstEdges.push_back(edge);
        }
    }

    std::cout << "MST edges: " << mstEdges.size()
              << ", Non-MST edges: " << nonMstEdges.size() << std::endl;

    // ========================================================================
    // Step 3: Keep MST + 40% of non-MST edges (remove 60%)
    // ========================================================================
    const double keepRatio = 0.6;  // Keep 60% of non-MST edges for reasonable density
    size_t nonMstToKeep = static_cast<size_t>(nonMstEdges.size() * keepRatio);

    // Shuffle non-MST edges and keep only a portion
    std::shuffle(nonMstEdges.begin(), nonMstEdges.end(), rng_);
    nonMstEdges.resize(nonMstToKeep);

    // ========================================================================
    // Step 4: Add final edges to graph with variable capacity
    // ========================================================================
    int edgesCreated = 0;
    double avgCellDim = (cellWidth + cellHeight) / 2.0;
    std::uniform_real_distribution<double> capacityNoise(-2.0, 2.0);

    auto setEdgeCapacity = [&](Edge::Id edgeId) {
        Edge* e = graph.getEdge(edgeId);
        if (e) {
            double lengthRatio = e->getLength() / avgCellDim;
            double capacity = 5.0 + 10.0 * lengthRatio + capacityNoise(rng_);
            e->setCapacity(std::max(3.0, capacity));
        }
    };

    // Add all MST edges (guarantees connectivity)
    for (const auto& edge : mstEdges) {
        Edge::Id eid = graph.addEdge(edge.source, edge.target);
        setEdgeCapacity(eid);
        edgesCreated++;
    }

    // Add kept non-MST edges (provides alternative paths)
    for (const auto& edge : nonMstEdges) {
        Edge::Id eid = graph.addEdge(edge.source, edge.target);
        setEdgeCapacity(eid);
        edgesCreated++;
    }

    std::cout << "Final graph: " << edgesCreated << " edges (sparsified from "
              << allEdges.size() << ")" << std::endl;
}

} // namespace nav
