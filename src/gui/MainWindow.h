#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <memory>
#include "core/graph/Graph.h"
#include "core/generation/GridPerturbationGenerator.h"
#include "core/algorithms/DijkstraPathFinder.h"
#include "core/algorithms/DynamicPathFinder.h"
#include "core/traffic/TrafficSimulator.h"
#include "core/spatial/QuadTree.h"
#include "gui/widgets/ControlPanel.h"

namespace nav {

class MapView;
class MapScene;

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

    // Load and display a graph
    void loadGraph(const Graph& graph);

    // Generate a new random map
    void generateNewMap(int numNodes = 10000, double width = 10000.0, double height = 10000.0);

private slots:
    void onGenerateMap();
    void onZoomToFit();
    void onToggleSimulation();
    void onSimulationStep();
    void onPathFound(const PathResult& result);
    void onStatusMessage(const QString& message);

    // Control panel slots
    void onFindNearestRequested(double x, double y, int k);
    void onComputePathRequested(uint32_t startId, uint32_t endId, RoutingCriteria criteria);
    void onShowTrafficNearRequested(double x, double y, double radius);
    void onClearHighlightsRequested();

private:
    void setupUi();
    void setupMenuBar();
    void setupToolBar();
    void setupControlPanel();
    void updatePathfinder();
    void rebuildQuadTree();

    MapView* mapView_;
    MapScene* mapScene_;
    ControlPanel* controlPanel_;

    std::unique_ptr<Graph> graph_;
    std::unique_ptr<GridPerturbationGenerator> generator_;
    std::unique_ptr<TrafficSimulator> simulator_;
    std::unique_ptr<DijkstraPathFinder> dijkstraPathfinder_;
    std::unique_ptr<DynamicPathFinder> dynamicPathfinder_;
    std::unique_ptr<QuadTree> quadTree_;

    QTimer* simulationTimer_;
    QAction* simulationAction_;
    bool simulationRunning_ = false;
    bool heatmapVisible_ = false;  // Toggle for visual heatmap display

    double mapWidth_ = 10000.0;
    double mapHeight_ = 10000.0;
};

} // namespace nav

#endif // MAINWINDOW_H
