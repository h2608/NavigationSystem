#ifndef CLUSTERITEM_H
#define CLUSTERITEM_H

#include <QGraphicsEllipseItem>
#include <QPainterPath>
#include <QRectF>

#include "core/graph/Node.h"

namespace nav {

class ClusterItem : public QGraphicsEllipseItem {
public:
    static constexpr double HITBOX_RADIUS = 15.0;

    ClusterItem(const Point2D& position, size_t memberCount,
                QGraphicsItem* parent = nullptr);

    size_t getMemberCount() const { return memberCount_; }
    bool beginVisualTransition(qreal targetOpacity);
    void applyVisualProgress(qreal progress);

    QRectF boundingRect() const override;
    QPainterPath shape() const override;
    void paint(QPainter* painter,
               const QStyleOptionGraphicsItem* option,
               QWidget* widget) override;

private:
    void refreshVisualState();

    size_t memberCount_;
    double visualRadius_;
    qreal startOpacity_ = 1.0;
    qreal currentOpacity_ = 1.0;
    qreal targetOpacity_ = 1.0;
    bool visualStateInitialized_ = false;
    qreal lastEffectiveOpacity_ = -1.0;
    bool lastVisible_ = true;
};

} // namespace nav

#endif // CLUSTERITEM_H
