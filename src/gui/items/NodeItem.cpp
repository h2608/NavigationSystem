#include "gui/items/NodeItem.h"

#include <algorithm>
#include <cmath>
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

bool NodeItem::beginVisualTransition(qreal targetOpacity) {
    startOpacity_ = currentOpacity_;
    targetOpacity_ = targetOpacity;
    return std::abs(targetOpacity_ - currentOpacity_) > 0.01;
}

void NodeItem::applyVisualProgress(qreal progress) {
    const qreal nextOpacity = (progress >= 1.0)
        ? targetOpacity_
        : lerp(startOpacity_, targetOpacity_, progress);
    if (std::abs(nextOpacity - currentOpacity_) < 0.01 && progress < 1.0) {
        return;
    }

    currentOpacity_ = nextOpacity;
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
    const bool visible = effectiveOpacity > 0.02 || highlighted_;

    bool needsUpdate = !visualStateInitialized_;
    if (!visualStateInitialized_ || std::abs(lastEffectiveOpacity_ - effectiveOpacity) >= 0.01) {
        setOpacity(effectiveOpacity);
        lastEffectiveOpacity_ = effectiveOpacity;
        needsUpdate = true;
    }

    if (!visualStateInitialized_ || lastVisible_ != visible) {
        setVisible(visible);
        lastVisible_ = visible;
        needsUpdate = true;
    }

    if (!visualStateInitialized_ || lastAppliedHighlighted_ != highlighted_) {
        lastAppliedHighlighted_ = highlighted_;
        needsUpdate = true;
    }

    visualStateInitialized_ = true;
    if (needsUpdate) {
        update();
    }
}

} // namespace nav
