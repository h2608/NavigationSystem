#ifndef NODEITEM_H
#define NODEITEM_H

#include <QGraphicsEllipseItem>
#include <QPainterPath>
#include <QRectF>

#include "core/graph/Node.h"

namespace nav {

class NodeItem : public QGraphicsEllipseItem {
public:
    static constexpr double NORMAL_RADIUS = 2.0;
    static constexpr double HIGHLIGHT_RADIUS = 8.0;
    static constexpr double HITBOX_RADIUS = 10.0;

    explicit NodeItem(Node::Id nodeId, const Point2D& position, QGraphicsItem* parent = nullptr);

    Node::Id getNodeId() const { return nodeId_; }
    void setHighlighted(bool highlighted);
    bool isHighlighted() const { return highlighted_; }
    bool isInteractive() const { return highlighted_; }
    bool beginVisualTransition(qreal targetOpacity);
    void applyVisualProgress(qreal progress);

    QRectF boundingRect() const override;
    QPainterPath shape() const override;
    void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;

private:
    void refreshVisualState();

    Node::Id nodeId_;
    bool highlighted_ = false;
    qreal startOpacity_ = 1.0;
    qreal currentOpacity_ = 1.0;
    qreal targetOpacity_ = 1.0;
    bool visualStateInitialized_ = false;
    qreal lastEffectiveOpacity_ = -1.0;
    bool lastVisible_ = true;
    bool lastAppliedHighlighted_ = false;
};

} // namespace nav

#endif // NODEITEM_H
