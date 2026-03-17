#include "gui/view/MapView.h"
#include "gui/view/MapScene.h"
#include "gui/items/NodeItem.h"
#include <QWheelEvent>
#include <QScrollBar>

namespace nav {

MapView::MapView(QWidget* parent)
    : QGraphicsView(parent)
{
    // Enable panning with mouse drag
    setDragMode(ScrollHandDrag);

    // Enable antialiasing for smoother rendering
    setRenderHint(QPainter::Antialiasing);
    setRenderHint(QPainter::SmoothPixmapTransform);

    // Optimize rendering
    setViewportUpdateMode(SmartViewportUpdate);
    setOptimizationFlag(DontSavePainterState);
    setOptimizationFlag(DontAdjustForAntialiasing);

    // Set anchor for transformations
    setTransformationAnchor(AnchorUnderMouse);
    setResizeAnchor(AnchorUnderMouse);

    // Allow scrolling beyond scene bounds slightly
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
}

void MapView::zoomToFit() {
    if (scene()) {
        fitInView(scene()->sceneRect(), Qt::KeepAspectRatio);
        currentZoom_ = transform().m11();  // Get current scale factor
        updateLOD();
    }
}

void MapView::updateLOD() {
    if (!scene()) return;

    MapScene* mapScene = qobject_cast<MapScene*>(scene());
    if (!mapScene) return;

    // Get current scale from transformation matrix
    double scale = transform().m11();

    // Show/hide nodes based on zoom level
    bool showNodes = (scale >= LOD_THRESHOLD);

    // Use the pre-built node map instead of iterating all scene items
    for (const auto& pair : mapScene->getNodeItems()) {
        if (pair.second) {
            pair.second->setVisible(showNodes);
        }
    }
}

void MapView::wheelEvent(QWheelEvent* event) {
    // Calculate zoom factor based on wheel delta
    double factor = (event->angleDelta().y() > 0) ? ZOOM_FACTOR : (1.0 / ZOOM_FACTOR);

    // Calculate new zoom level
    double newZoom = currentZoom_ * factor;

    // Clamp zoom level
    if (newZoom < MIN_ZOOM || newZoom > MAX_ZOOM) {
        return;
    }

    // Apply zoom centered on cursor position
    scale(factor, factor);
    currentZoom_ = newZoom;

    // Update LOD after zooming
    updateLOD();
}

void MapView::focusOnPoint(double x, double y, double zoomLevel) {
    // Reset transform and set desired zoom level
    resetTransform();
    scale(zoomLevel, zoomLevel);
    currentZoom_ = zoomLevel;

    // Center on the point
    centerOn(x, y);

    // Update LOD
    updateLOD();
}

void MapView::focusOnBounds(const QRectF& bounds, double padding) {
    if (bounds.isEmpty()) return;

    // Add padding to bounds
    QRectF paddedBounds = bounds.adjusted(-padding, -padding, padding, padding);

    // Fit the view to the padded bounds
    fitInView(paddedBounds, Qt::KeepAspectRatio);
    currentZoom_ = transform().m11();

    // Update LOD
    updateLOD();
}

} // namespace nav
