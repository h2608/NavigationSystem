#ifndef MAPVIEW_H
#define MAPVIEW_H

#include <QElapsedTimer>
#include <QGraphicsView>
#include <QPointF>
#include <QRectF>
#include <QTimer>

#include "gui/view/DisplayFilter.h"

namespace nav {

class MapView : public QGraphicsView {
    Q_OBJECT

public:
    explicit MapView(QWidget* parent = nullptr);

    void zoomToFit();
    void updateLOD();
    double getCurrentZoom() const { return currentZoom_; }
    void focusOnPoint(double x, double y, double zoomLevel = 2.0);
    void focusOnBounds(const QRectF& bounds, double padding = 50.0);

protected:
    void wheelEvent(QWheelEvent* event) override;

private:
    void scheduleLODUpdate();
    void startCameraAnimation(const QPointF& targetCenter, double targetZoom, int durationMs);
    void stopCameraAnimation(bool snapToTarget = false);
    void advanceCameraAnimation();
    void applyCameraFrame(double zoom, const QPointF& center);
    QPointF currentViewCenter() const;
    double computeZoomForBounds(const QRectF& bounds) const;

    static constexpr double ZOOM_FACTOR = 1.15;
    static constexpr double MIN_ZOOM = 0.01;
    static constexpr double MAX_ZOOM = 100.0;
    static constexpr int LOD_THROTTLE_MS = 24;
    static constexpr int CAMERA_FRAME_MS = 16;

    DisplayFilter displayFilter_;
    double currentZoom_ = 1.0;
    int currentBandIndex_ = -1;

    QTimer lodUpdateTimer_;
    QTimer cameraAnimationTimer_;
    QElapsedTimer cameraAnimationClock_;
    QPointF cameraStartCenter_;
    QPointF cameraTargetCenter_;
    double cameraStartZoom_ = 1.0;
    double cameraTargetZoom_ = 1.0;
    int cameraDurationMs_ = 180;
};

} // namespace nav

#endif // MAPVIEW_H
