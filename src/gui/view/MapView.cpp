#include "gui/view/MapView.h"

#include <QPainter>
#include <QScrollBar>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QEasingCurve>

#include <algorithm>
#include <cmath>

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

    lodApplyTimer_.setSingleShot(true);
    connect(&lodApplyTimer_, &QTimer::timeout, this, &MapView::applyDeferredLODUpdate);

    zoomFrameTimer_.setSingleShot(true);
    connect(&zoomFrameTimer_, &QTimer::timeout, this, &MapView::applyPendingZoomFrame);

    cameraAnimationTimer_.setInterval(CAMERA_FRAME_MS);
    connect(&cameraAnimationTimer_, &QTimer::timeout, this, &MapView::advanceCameraAnimation);

    interactionEndTimer_.setSingleShot(true);
    connect(&interactionEndTimer_, &QTimer::timeout, this, &MapView::finishInteraction);
}

void MapView::zoomToFit() {
    if (!scene() || scene()->sceneRect().isEmpty()) return;

    startCameraAnimation(scene()->sceneRect().center(),
                         computeZoomForBounds(scene()->sceneRect()),
                         200);
}

void MapView::zoomIn() {
    requestZoomByFactor(ZOOM_FACTOR, currentViewCenter());
}

void MapView::zoomOut() {
    requestZoomByFactor(1.0 / ZOOM_FACTOR, currentViewCenter());
}

void MapView::updateLOD() {
    if (!scene()) return;

    currentZoom_ = transform().m11();

    MapScene* mapScene = qobject_cast<MapScene*>(scene());
    if (!mapScene) return;

    const bool hasAppliedBand = mapScene->hasDisplayBand();
    int nextBandIndex = displayFilter_.getBandIndex(
        currentZoom_,
        hasAppliedBand ? currentBandIndex_ : -1
    );
    if (nextBandIndex < 0) {
        nextBandIndex = 0;
    }

    mapScene->updateViewZoom(currentZoom_);

    const bool needsInitialRefresh = mapScene->needsDisplayBandRefresh();
    if (nextBandIndex == currentBandIndex_ && !needsInitialRefresh) {
        return;
    }

    if (interactionActive_ && hasAppliedBand) {
        return;
    }

    applyDisplayBand(mapScene, nextBandIndex);
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
    const double factor = (event->angleDelta().y() > 0) ? ZOOM_FACTOR : (1.0 / ZOOM_FACTOR);
    requestZoomByFactor(factor, mapToScene(event->pos()));
    event->accept();
}

void MapView::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        beginInteraction();
    }
    QGraphicsView::mousePressEvent(event);
}

void MapView::mouseReleaseEvent(QMouseEvent* event) {
    QGraphicsView::mouseReleaseEvent(event);
    if (event->button() == Qt::LeftButton) {
        scheduleInteractionEnd();
    }
}

void MapView::beginInteraction() {
    interactionEndTimer_.stop();
    lodApplyTimer_.stop();
    interactionEndSignalPending_ = false;
    enableInteractiveRenderMode();

    if (MapScene* mapScene = qobject_cast<MapScene*>(scene())) {
        mapScene->cancelDisplayTransition();
    }

    if (interactionActive_) {
        return;
    }

    interactionActive_ = true;
    emit interactionBegan();
}

void MapView::scheduleInteractionEnd() {
    if (!interactionActive_) {
        return;
    }
    interactionEndTimer_.start(INTERACTION_IDLE_MS);
}

void MapView::finishInteraction() {
    if (!interactionActive_) {
        return;
    }

    interactionActive_ = false;
    interactionEndSignalPending_ = true;
    if (zoomFrameTimer_.isActive()) {
        zoomFrameTimer_.stop();
    }
    if (pendingZoomActive_) {
        applyPendingZoomFrame();
    }
    if (lodUpdateTimer_.isActive()) {
        lodUpdateTimer_.stop();
    }
    scheduleDeferredLODUpdate();
}

void MapView::scheduleLODUpdate() {
    if (!lodUpdateTimer_.isActive()) {
        lodUpdateTimer_.start(LOD_THROTTLE_MS);
    }
}

void MapView::scheduleDeferredLODUpdate() {
    if (lodApplyTimer_.isActive()) {
        lodApplyTimer_.stop();
    }
    lodApplyTimer_.start(LOD_APPLY_DELAY_MS);
}

void MapView::applyDeferredLODUpdate() {
    updateLOD();

    MapScene* mapScene = qobject_cast<MapScene*>(scene());
    if (!mapScene || !mapScene->hasPendingDisplayWork()) {
        onSceneDisplayUpdateFinished();
    }
}

void MapView::applyDisplayBand(MapScene* mapScene, int bandIndex) {
    if (!mapScene) return;

    currentBandIndex_ = bandIndex;
    connect(mapScene, &MapScene::displayUpdateFinished,
            this, &MapView::onSceneDisplayUpdateFinished,
            Qt::UniqueConnection);
    mapScene->requestDisplayBandTransition(
        currentBandIndex_,
        displayFilter_.getBand(currentBandIndex_),
        currentViewportSceneRect().adjusted(-160.0, -160.0, 160.0, 160.0)
    );
}

void MapView::requestZoomByFactor(double factor, const QPointF& anchor) {
    beginInteraction();

    if (cameraAnimationTimer_.isActive()) {
        stopCameraAnimation(false);
    }

    const double boundedZoom = std::clamp(currentZoom_ * pendingZoomFactor_ * factor,
                                          MIN_ZOOM,
                                          MAX_ZOOM);
    pendingZoomFactor_ = boundedZoom / currentZoom_;
    pendingZoomAnchor_ = anchor;
    pendingZoomActive_ = true;

    if (!zoomFrameTimer_.isActive()) {
        zoomFrameTimer_.start(ZOOM_FRAME_MS);
    }
    scheduleInteractionEnd();
}

void MapView::applyPendingZoomFrame() {
    if (!pendingZoomActive_) {
        return;
    }

    const double oldZoom = currentZoom_;
    const double newZoom = std::clamp(oldZoom * pendingZoomFactor_, MIN_ZOOM, MAX_ZOOM);
    const QPointF anchor = pendingZoomAnchor_;
    pendingZoomFactor_ = 1.0;
    pendingZoomActive_ = false;

    if (std::abs(newZoom - oldZoom) < 0.000001) {
        return;
    }

    const QPointF oldCenter = currentViewCenter();
    const QPointF offset = oldCenter - anchor;
    const double centerScale = oldZoom / newZoom;
    const QPointF newCenter(
        anchor.x() + offset.x() * centerScale,
        anchor.y() + offset.y() * centerScale
    );

    QTransform transform;
    transform.scale(newZoom, newZoom);
    setTransform(transform);
    centerOn(newCenter);
    currentZoom_ = transform.m11();
    scheduleLODUpdate();
}

void MapView::enableInteractiveRenderMode() {
    if (interactiveRenderMode_) {
        return;
    }

    steadyAntialiasing_ = renderHints().testFlag(QPainter::Antialiasing);
    steadySmoothPixmapTransform_ = renderHints().testFlag(QPainter::SmoothPixmapTransform);
    steadyViewportUpdateMode_ = viewportUpdateMode();
    setRenderHint(QPainter::Antialiasing, false);
    setRenderHint(QPainter::SmoothPixmapTransform, false);
    setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
    interactiveRenderMode_ = true;
}

void MapView::restoreInteractiveRenderMode() {
    if (interactiveRenderMode_) {
        setRenderHint(QPainter::Antialiasing, steadyAntialiasing_);
        setRenderHint(QPainter::SmoothPixmapTransform, steadySmoothPixmapTransform_);
        setViewportUpdateMode(steadyViewportUpdateMode_);
        viewport()->update();
        interactiveRenderMode_ = false;
    }

    if (interactionEndSignalPending_) {
        interactionEndSignalPending_ = false;
        emit interactionEnded();
    }
}

void MapView::onSceneDisplayUpdateFinished() {
    if (interactionActive_) {
        return;
    }
    restoreInteractiveRenderMode();
}

void MapView::startCameraAnimation(const QPointF& targetCenter, double targetZoom, int durationMs) {
    beginInteraction();
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
    scheduleInteractionEnd();
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

QRectF MapView::currentViewportSceneRect() const {
    return mapToScene(viewport()->rect()).boundingRect();
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
