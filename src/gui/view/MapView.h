#ifndef MAPVIEW_H
#define MAPVIEW_H

#include <QElapsedTimer>
#include <QGraphicsView>
#include <QPointF>
#include <QRectF>
#include <QTimer>

#include "gui/view/DisplayFilter.h"

class QMouseEvent;
class QWheelEvent;

namespace nav {

class MapScene;

class MapView : public QGraphicsView {
    Q_OBJECT

public:
    explicit MapView(QWidget* parent = nullptr);

    void zoomToFit();
    void zoomIn();
    void zoomOut();
    void updateLOD();
    double getCurrentZoom() const { return currentZoom_; }
    bool isInteractionActive() const { return interactionActive_; }
    void focusOnPoint(double x, double y, double zoomLevel = 2.0);
    void focusOnBounds(const QRectF& bounds, double padding = 50.0);

signals:
    void interactionBegan();
    void interactionEnded();

protected:
    void wheelEvent(QWheelEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;

private:
    void beginInteraction();
    void scheduleInteractionEnd();
    void finishInteraction();
    void scheduleLODUpdate();
    void scheduleDeferredLODUpdate();
    void applyDeferredLODUpdate();
    void applyDisplayBand(MapScene* mapScene, int bandIndex);
    void requestZoomByFactor(double factor, const QPointF& anchor);
    void applyPendingZoomFrame();
    void enableInteractiveRenderMode();
    void restoreInteractiveRenderMode();
    void onSceneDisplayUpdateFinished();
    void startCameraAnimation(const QPointF& targetCenter, double targetZoom, int durationMs);
    void stopCameraAnimation(bool snapToTarget = false);
    void advanceCameraAnimation();
    void applyCameraFrame(double zoom, const QPointF& center);
    QPointF currentViewCenter() const;
    QRectF currentViewportSceneRect() const;
    double computeZoomForBounds(const QRectF& bounds) const;

    static constexpr double ZOOM_FACTOR = 1.15;
    static constexpr double MIN_ZOOM = 0.01;
    static constexpr double MAX_ZOOM = 100.0;
    static constexpr int LOD_THROTTLE_MS = 24;
    static constexpr int LOD_APPLY_DELAY_MS = 20;
    static constexpr int CAMERA_FRAME_MS = 16;
    static constexpr int ZOOM_FRAME_MS = 16;
    static constexpr int INTERACTION_IDLE_MS = 450;

    DisplayFilter displayFilter_;
    double currentZoom_ = 1.0;
    int currentBandIndex_ = -1;
    bool interactionActive_ = false;
    bool interactionEndSignalPending_ = false;
    bool pendingZoomActive_ = false;
    bool interactiveRenderMode_ = false;
    bool steadyAntialiasing_ = true;
    bool steadySmoothPixmapTransform_ = true;
    double pendingZoomFactor_ = 1.0;
    QPointF pendingZoomAnchor_;
    ViewportUpdateMode steadyViewportUpdateMode_ = SmartViewportUpdate;

    QTimer lodUpdateTimer_;
    QTimer lodApplyTimer_;
    QTimer zoomFrameTimer_;
    QTimer cameraAnimationTimer_;
    QTimer interactionEndTimer_;
    QElapsedTimer cameraAnimationClock_;
    QPointF cameraStartCenter_;
    QPointF cameraTargetCenter_;
    double cameraStartZoom_ = 1.0;
    double cameraTargetZoom_ = 1.0;
    int cameraDurationMs_ = 180;
};

} // namespace nav

#endif // MAPVIEW_H
