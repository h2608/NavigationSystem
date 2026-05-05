# Core library (GUI-independent logic)
add_library(NavigationCore STATIC
    core/graph/Node.cpp
    core/graph/Edge.cpp
    core/graph/Graph.cpp
    core/io/GraphIO.cpp
    core/spatial/BoundingBox.cpp
    core/spatial/QuadTree.cpp
    core/spatial/ClusterIndex.cpp
    core/generation/GridPerturbationGenerator.cpp
    core/generation/HierarchicalRoadGenerator.cpp
    core/algorithms/DijkstraPathFinder.cpp
    core/algorithms/DynamicPathFinder.cpp
    core/traffic/TrafficModel.cpp
    core/traffic/TrafficSimulator.cpp
)

target_include_directories(NavigationCore PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(NavigationCore PUBLIC
    Qt5::Core
)

# GUI library
add_library(NavigationGui STATIC
    gui/items/NodeItem.cpp
    gui/items/EdgeItem.cpp
    gui/items/ClusterItem.cpp
    gui/view/MapScene.cpp
    gui/view/MapView.cpp
    gui/widgets/ControlPanel.cpp
    gui/widgets/GenerateMapDialog.cpp
    gui/MainWindow.cpp
)

target_include_directories(NavigationGui PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(NavigationGui PUBLIC
    NavigationCore
    Qt5::Widgets
)

# Main application executable
add_executable(NavigationSystem
    app/main.cpp
)

target_link_libraries(NavigationSystem PRIVATE
    NavigationGui
    Qt5::Widgets
)
