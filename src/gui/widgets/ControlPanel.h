#ifndef CONTROLPANEL_H
#define CONTROLPANEL_H

#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QPushButton>
#include <QLabel>
#include <QComboBox>
#include "core/graph/Node.h"
#include <vector>

namespace nav {

// Routing criteria enum
enum class RoutingCriteria {
    ShortestDistance = 0,  // F2: Uses DijkstraPathFinder (distance-based)
    FastestTime = 1        // F4: Uses DynamicPathFinder (traffic-aware)
};

class ControlPanel : public QDockWidget {
    Q_OBJECT

public:
    explicit ControlPanel(QWidget* parent = nullptr);

    // Set valid ranges for node IDs
    void setNodeIdRange(int minId, int maxId);

    // Set coordinate ranges
    void setCoordinateRange(double minX, double maxX, double minY, double maxY);

    // Get current routing criteria
    RoutingCriteria getRoutingCriteria() const;

signals:
    // Spatial query signals
    void findNearestRequested(double x, double y, int k);

    // Pathfinding signals (now includes routing criteria)
    void computePathRequested(uint32_t startId, uint32_t endId, RoutingCriteria criteria);

    // Traffic view signals
    void showTrafficNearRequested(double x, double y, double radius);

    // Clear signals
    void clearHighlightsRequested();

public slots:
    // Update result display
    void showSpatialQueryResult(int nodeCount, int edgeCount);
    void showPathResult(int nodeCount, double totalCost, bool found);
    void showTrafficResult(int edgeCount);

private slots:
    void onFindNearestClicked();
    void onComputePathClicked();
    void onShowTrafficClicked();
    void onClearClicked();

private:
    void setupUi();

    // Spatial Query section
    QDoubleSpinBox* xCoordSpinBox_;
    QDoubleSpinBox* yCoordSpinBox_;
    QSpinBox* kNearestSpinBox_;
    QPushButton* findNearestButton_;
    QLabel* spatialResultLabel_;

    // Path by ID section
    QSpinBox* startNodeSpinBox_;
    QSpinBox* endNodeSpinBox_;
    QComboBox* routingCriteriaCombo_;
    QPushButton* computePathButton_;
    QLabel* pathResultLabel_;

    // Traffic view section
    QDoubleSpinBox* trafficXSpinBox_;
    QDoubleSpinBox* trafficYSpinBox_;
    QDoubleSpinBox* trafficRadiusSpinBox_;
    QPushButton* showTrafficButton_;
    QLabel* trafficResultLabel_;

    // Clear button
    QPushButton* clearButton_;
};

} // namespace nav

#endif // CONTROLPANEL_H
