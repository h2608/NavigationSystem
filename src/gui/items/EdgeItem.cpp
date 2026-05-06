#include "gui/items/EdgeItem.h"

#include <cmath>
#include <QPainter>
#include <QPen>

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
    , baseStyle_(RoadStyle::forTier(displayTier, roadClass, importanceScore))
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

bool EdgeItem::beginVisualTransition(qreal targetOpacity, double targetWidthScale) {
    static constexpr qreal kOpacityEpsilon = 0.01;
    static constexpr double kWidthEpsilon = 0.04;

    startOpacity_ = currentOpacity_;
    startWidthScale_ = currentWidthScale_;
    targetOpacity_ = targetOpacity;
    targetWidthScale_ = targetWidthScale;

    const bool widthChanged = std::abs(targetWidthScale_ - currentWidthScale_) > kWidthEpsilon;
    const bool opacityChanged = std::abs(targetOpacity_ - currentOpacity_) > kOpacityEpsilon;
    return opacityChanged || widthChanged;
}

void EdgeItem::applyVisualProgress(qreal progress) {
    static constexpr qreal kOpacityEpsilon = 0.01;
    static constexpr double kWidthEpsilon = 0.04;

    const qreal nextOpacity = (progress >= 1.0)
        ? targetOpacity_
        : static_cast<qreal>(lerp(startOpacity_, targetOpacity_, progress));
    const double nextWidthScale = (progress >= 1.0)
        ? targetWidthScale_
        : lerp(startWidthScale_, targetWidthScale_, progress);

    if (std::abs(nextOpacity - currentOpacity_) < kOpacityEpsilon &&
        std::abs(nextWidthScale - currentWidthScale_) < kWidthEpsilon &&
        progress < 1.0) {
        return;
    }

    currentOpacity_ = nextOpacity;
    currentWidthScale_ = nextWidthScale;
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
    static constexpr double kWidthEpsilon = 0.04;
    static constexpr double kZValueEpsilon = 0.01;

    const qreal effectiveOpacity =
        (spatialHighlighted_ || trafficFocused_ || congestionStatus_ > 0)
            ? std::max<qreal>(currentOpacity_, 1.0)
            : currentOpacity_;
    const bool visible = effectiveOpacity > 0.02 || spatialHighlighted_ || trafficFocused_ || congestionStatus_ > 0;
    const QColor boundingPenColor = currentCasingColor();
    const double boundingPenWidth = currentOuterWidth();

    double zValue = baseStyle_.zValue;
    if (trafficFocused_) zValue += 1.0;
    if (spatialHighlighted_) zValue += 2.0;

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

    if (!visualStateInitialized_ ||
        lastBoundingPenColor_ != boundingPenColor ||
        std::abs(lastBoundingPenWidth_ - boundingPenWidth) >= kWidthEpsilon) {
        setPen(QPen(boundingPenColor, boundingPenWidth, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        lastBoundingPenColor_ = boundingPenColor;
        lastBoundingPenWidth_ = boundingPenWidth;
        needsUpdate = true;
    }

    if (!visualStateInitialized_ || std::abs(lastZValue_ - zValue) >= kZValueEpsilon) {
        setZValue(zValue);
        lastZValue_ = zValue;
        needsUpdate = true;
    }

    visualStateInitialized_ = true;
    if (needsUpdate) {
        update();
    }
}

QColor EdgeItem::currentFillColor() const {
    if (spatialHighlighted_ && spatialHighlightColor_.isValid()) {
        return spatialHighlightColor_;
    }
    if (congestionStatus_ > 0) {
        return RoadStyle::congestionColor(congestionStatus_);
    }
    return baseStyle_.fillColor;
}

QColor EdgeItem::currentCasingColor() const {
    if (spatialHighlighted_ && spatialHighlightColor_.isValid()) {
        return spatialHighlightColor_.darker(155);
    }
    if (congestionStatus_ > 0) {
        return RoadStyle::congestionColor(congestionStatus_).darker(150);
    }
    return baseStyle_.casingColor;
}

double EdgeItem::currentInnerWidth() const {
    double width = baseStyle_.width * currentWidthScale_;
    if (spatialHighlighted_) width += 0.6;
    if (trafficFocused_) width += 0.25;
    width += RoadStyle::congestionWidthBoost(congestionStatus_);
    return width;
}

double EdgeItem::currentOuterWidth() const {
    double width = baseStyle_.casingWidth * currentWidthScale_;
    if (spatialHighlighted_) width += 0.8;
    if (trafficFocused_) width += 0.35;
    width += RoadStyle::congestionWidthBoost(congestionStatus_);
    return width;
}

} // namespace nav
