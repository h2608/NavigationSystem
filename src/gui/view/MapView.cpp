#include "gui/view/MapView.h"

#include <QPainter>
#include <QScrollBar>
#include <QWheelEvent>
#include <QEasingCurve>

#include <algorithm>

#include "gui/view/MapScene.h"

namespace nav {

namespace {

QPointF lerpPoint(const QPointF& start, const QPointF& end, qreal progress) {
    return QPointF(
        start.x() + (end.x() - start.x()) * progress,
        start.y() + (end.y() - start.y()) * progress
    );
}

double lerpDouble(double start, double end, qreal progress) {
    return start + (end - start) * progress;
}

} // namespace

MapView::MapView(QWidget* parent)
    : QGraphicsView(parent)
{
    setDragMode(ScrollHandDrag);
    setRenderHint(QPainter::Antialiasing);
    setRenderHint(QPainter::SmoothPixmapTransform);
    setViewportUpdateMode(SmartViewportUpdate);
    setOptimizationFlag(DontSavePainterState);
    setOptimizationFlag(DontAdjustForAntialiasing);
    setTransformationAnchor(AnchorUnderMouse);
    setResizeAnchor(AnchorUnderMouse);
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    lodUpdateTimer_.setSingleShot(true);
    connect(&lodUpdateTimer_, &QTimer::timeout, this, &MapView::updateLOD);

    cameraAnimationTimer_.setInterval(CAMERA_FRAME_MS);
    connect(&cameraAnimationTimer_, &QTimer::timeout, this, &MapView::advanceCameraAnimation);
}

void MapView::zoomToFit() {
    if (!scene() || scene()->sceneRect().isEmpty()) return;

    startCameraAnimation(scene()->sceneRect().center(),
                         computeZoomForBounds(scene()->sceneRect()),
                         200);
}

void MapView::updateLOD() {
    if (!scene()) return;

    currentZoom_ = transform().m11();

    MapScene* mapScene = qobject_cast<MapScene*>(scene());
    if (!mapScene) return;

    currentBandIndex_ = displayFilter_.getBandIndex(currentZoom_, currentBandIndex_);
    if (currentBandIndex_ < 0) {
        currentBandIndex_ = 0;
    }

    mapScene->transitionToDisplayBand(displayFilter_.getBand(currentBandIndex_), currentZoom_);
}

void MapView::focusOnPoint(double x, double y, double zoomLevel) {
    startCameraAnimation(QPointF(x, y),
                         std::clamp(zoomLevel, MIN_ZOOM, MAX_ZOOM),
                         170);
}

void MapView::focusOnBounds(const QRectF& bounds, double padding) {
    if (bounds.isEmpty()) return;

    const QRectF paddedBounds = bounds.adjusted(-padding, -padding, padding, padding);
    startCameraAnimation(paddedBounds.center(),
                         computeZoomForBounds(paddedBounds),
                         190);
}

void MapView::wheelEvent(QWheelEvent* event) {
    if (cameraAnimationTimer_.isActive()) {
        stopCameraAnimation(false);
    }

    const double factor = (event->angleDelta().y() > 0) ? ZOOM_FACTOR : (1.0 / ZOOM_FACTOR);
    const double newZoom = currentZoom_ * factor;
    if (newZoom < MIN_ZOOM || newZoom > MAX_ZOOM) {
        event->accept();
        return;
    }

    scale(factor, factor);
    currentZoom_ = transform().m11();
    scheduleLODUpdate();
    event->accept();
}

void MapView::scheduleLODUpdate() {
    if (!lodUpdateTimer_.isActive()) {
        lodUpdateTimer_.start(LOD_THROTTLE_MS);
    }
}

void MapView::startCameraAnimation(const QPointF& targetCenter, double targetZoom, int durationMs) {
    cameraStartCenter_ = currentViewCenter();
    cameraTargetCenter_ = targetCenter;
    cameraStartZoom_ = currentZoom_;
    cameraTargetZoom_ = std::clamp(targetZoom, MIN_ZOOM, MAX_ZOOM);
    cameraDurationMs_ = durationMs;
    cameraAnimationClock_.restart();

    if (!cameraAnimationTimer_.isActive()) {
        cameraAnimationTimer_.start();
    }

    scheduleLODUpdate();
}

void MapView::stopCameraAnimation(bool snapToTarget) {
    if (snapToTarget) {
        applyCameraFrame(cameraTargetZoom_, cameraTargetCenter_);
    }
    cameraAnimationTimer_.stop();
}

void MapView::advanceCameraAnimation() {
    if (!cameraAnimationClock_.isValid()) {
        cameraAnimationTimer_.stop();
        return;
    }

    const qreal progress = std::clamp(
        static_cast<qreal>(cameraAnimationClock_.elapsed()) /
        static_cast<qreal>(cameraDurationMs_),
        0.0,
        1.0
    );
    const qreal eased = QEasingCurve(QEasingCurve::OutCubic).valueForProgress(progress);

    applyCameraFrame(
        lerpDouble(cameraStartZoom_, cameraTargetZoom_, eased),
        lerpPoint(cameraStartCenter_, cameraTargetCenter_, eased)
    );

    if (progress >= 1.0) {
        stopCameraAnimation(true);
    }
}

void MapView::applyCameraFrame(double zoom, const QPointF& center) {
    QTransform transform;
    transform.scale(zoom, zoom);
    setTransform(transform);
    centerOn(center);
    currentZoom_ = zoom;
    scheduleLODUpdate();
}

QPointF MapView::currentViewCenter() const {
    return mapToScene(viewport()->rect().center());
}

double MapView::computeZoomForBounds(const QRectF& bounds) const {
    if (bounds.isEmpty() || viewport()->width() <= 0 || viewport()->height() <= 0) {
        return currentZoom_;
    }

    const double xScale = viewport()->width() / bounds.width();
    const double yScale = viewport()->height() / bounds.height();
    return std::clamp(std::min(xScale, yScale), MIN_ZOOM, MAX_ZOOM);
}

} // namespace nav
