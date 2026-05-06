#include "gui/items/EdgeItem.h"

#include <QPainter>
#include <QPen>

#include "gui/items/RoadStyle.h"

namespace nav {

namespace {

double lerp(double start, double end, qreal progress) {
    return start + (end - start) * progress;
}

} // namespace

EdgeItem::EdgeItem(Edge::Id edgeId, const Point2D& from, const Point2D& to,
                   RoadClass roadClass, DisplayTier displayTier,
                   double importanceScore, QGraphicsItem* parent)
    : QGraphicsLineItem(parent)
    , edgeId_(edgeId)
    , roadClass_(roadClass)
    , displayTier_(displayTier)
    , importanceScore_(importanceScore)
{
    setLine(from.x, from.y, to.x, to.y);
    applyBaseStyle();
}

void EdgeItem::updateStyle(int congestionStatus) {
    congestionStatus_ = congestionStatus;
    refreshVisualState();
}

void EdgeItem::applyBaseStyle() {
    congestionStatus_ = 0;
    refreshVisualState();
}

void EdgeItem::setSpatialHighlight(bool highlighted, const QColor& color) {
    spatialHighlighted_ = highlighted;
    if (highlighted && color.isValid()) {
        spatialHighlightColor_ = color;
    }
    refreshVisualState();
}

void EdgeItem::setTrafficFocus(bool focused) {
    trafficFocused_ = focused;
    refreshVisualState();
}

void EdgeItem::beginVisualTransition(qreal targetOpacity, double targetWidthScale) {
    startOpacity_ = currentOpacity_;
    startWidthScale_ = currentWidthScale_;
    targetOpacity_ = targetOpacity;
    targetWidthScale_ = targetWidthScale;
}

void EdgeItem::applyVisualProgress(qreal progress) {
    currentOpacity_ = static_cast<qreal>(lerp(startOpacity_, targetOpacity_, progress));
    currentWidthScale_ = lerp(startWidthScale_, targetWidthScale_, progress);
    refreshVisualState();
}

void EdgeItem::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) {
    Q_UNUSED(option);
    Q_UNUSED(widget);

    painter->setRenderHint(QPainter::Antialiasing, true);

    QPen outerPen(currentCasingColor(), currentOuterWidth(), Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    QPen innerPen(currentFillColor(), currentInnerWidth(), Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);

    painter->setPen(outerPen);
    painter->drawLine(line());
    painter->setPen(innerPen);
    painter->drawLine(line());
}

void EdgeItem::refreshVisualState() {
    const qreal effectiveOpacity =
        (spatialHighlighted_ || trafficFocused_ || congestionStatus_ > 0)
            ? std::max<qreal>(currentOpacity_, 1.0)
            : currentOpacity_;

    setOpacity(effectiveOpacity);
    setVisible(effectiveOpacity > 0.02 || spatialHighlighted_ || trafficFocused_ || congestionStatus_ > 0);
    setPen(QPen(currentCasingColor(), currentOuterWidth(), Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));

    const RoadStyle style = RoadStyle::forTier(displayTier_, roadClass_, importanceScore_);
    double zValue = style.zValue;
    if (trafficFocused_) zValue += 1.0;
    if (spatialHighlighted_) zValue += 2.0;
    setZValue(zValue);

    update();
}

QColor EdgeItem::currentFillColor() const {
    if (spatialHighlighted_ && spatialHighlightColor_.isValid()) {
        return spatialHighlightColor_;
    }
    if (congestionStatus_ > 0) {
        return RoadStyle::congestionColor(congestionStatus_);
    }
    return RoadStyle::forTier(displayTier_, roadClass_, importanceScore_).fillColor;
}

QColor EdgeItem::currentCasingColor() const {
    if (spatialHighlighted_ && spatialHighlightColor_.isValid()) {
        return spatialHighlightColor_.darker(155);
    }
    if (congestionStatus_ > 0) {
        return RoadStyle::congestionColor(congestionStatus_).darker(150);
    }
    return RoadStyle::forTier(displayTier_, roadClass_, importanceScore_).casingColor;
}

double EdgeItem::currentInnerWidth() const {
    const RoadStyle style = RoadStyle::forTier(displayTier_, roadClass_, importanceScore_);
    double width = style.width * currentWidthScale_;
    if (spatialHighlighted_) width += 0.6;
    if (trafficFocused_) width += 0.25;
    width += RoadStyle::congestionWidthBoost(congestionStatus_);
    return width;
}

double EdgeItem::currentOuterWidth() const {
    const RoadStyle style = RoadStyle::forTier(displayTier_, roadClass_, importanceScore_);
    double width = style.casingWidth * currentWidthScale_;
    if (spatialHighlighted_) width += 0.8;
    if (trafficFocused_) width += 0.35;
    width += RoadStyle::congestionWidthBoost(congestionStatus_);
    return width;
}

} // namespace nav
