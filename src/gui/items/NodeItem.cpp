#include "gui/items/NodeItem.h"

#include <algorithm>
#include <QBrush>
#include <QPainter>
#include <QPen>

namespace nav {

namespace {

qreal lerp(qreal start, qreal end, qreal progress) {
    return start + (end - start) * progress;
}

} // namespace

NodeItem::NodeItem(Node::Id nodeId, const Point2D& position, QGraphicsItem* parent)
    : QGraphicsEllipseItem(parent)
    , nodeId_(nodeId)
{
    setPos(position.x, position.y);
    setBrush(QBrush(QColor(66, 72, 78)));
    setPen(Qt::NoPen);
    setZValue(10.0);
    setAcceptHoverEvents(true);
    setFlag(QGraphicsItem::ItemIsSelectable, false);
}

void NodeItem::setHighlighted(bool highlighted) {
    highlighted_ = highlighted;
    refreshVisualState();
}

void NodeItem::beginVisualTransition(qreal targetOpacity) {
    startOpacity_ = currentOpacity_;
    targetOpacity_ = targetOpacity;
}

void NodeItem::applyVisualProgress(qreal progress) {
    currentOpacity_ = lerp(startOpacity_, targetOpacity_, progress);
    refreshVisualState();
}

QRectF NodeItem::boundingRect() const {
    return QRectF(-HITBOX_RADIUS, -HITBOX_RADIUS,
                  HITBOX_RADIUS * 2.0, HITBOX_RADIUS * 2.0);
}

QPainterPath NodeItem::shape() const {
    QPainterPath path;
    path.addEllipse(-HITBOX_RADIUS, -HITBOX_RADIUS,
                    HITBOX_RADIUS * 2.0, HITBOX_RADIUS * 2.0);
    return path;
}

void NodeItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) {
    Q_UNUSED(option);
    Q_UNUSED(widget);

    const double baseRadius = highlighted_ ? HIGHLIGHT_RADIUS : NORMAL_RADIUS;
    const double radius = highlighted_
        ? baseRadius
        : baseRadius * (0.75 + static_cast<double>(std::max<qreal>(currentOpacity_, 0.15)) * 0.35);

    painter->setRenderHint(QPainter::Antialiasing, true);
    painter->setBrush(brush());
    painter->setPen(pen());
    painter->drawEllipse(QRectF(-radius, -radius, radius * 2.0, radius * 2.0));
}

void NodeItem::refreshVisualState() {
    const qreal effectiveOpacity = highlighted_ ? 1.0 : currentOpacity_;
    setOpacity(effectiveOpacity);
    setVisible(effectiveOpacity > 0.02 || highlighted_);
    update();
}

} // namespace nav
