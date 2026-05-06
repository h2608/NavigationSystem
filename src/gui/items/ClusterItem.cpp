#include "gui/items/ClusterItem.h"

#include <QFont>
#include <QPainter>
#include <algorithm>
#include <cmath>

namespace nav {

namespace {

qreal lerp(qreal start, qreal end, qreal progress) {
    return start + (end - start) * progress;
}

} // namespace

ClusterItem::ClusterItem(const Point2D& position, size_t memberCount,
                         QGraphicsItem* parent)
    : QGraphicsEllipseItem(parent)
    , memberCount_(memberCount)
    , visualRadius_(std::max(6.0, std::min(12.0, 4.0 + std::log2(static_cast<double>(memberCount)))))
{
    setPos(position.x, position.y);
    setFlag(QGraphicsItem::ItemIgnoresTransformations, true);
    setBrush(QBrush(QColor(82, 120, 168, 185)));
    setPen(QPen(QColor(55, 84, 124), 1.2));
    setZValue(11.0);
    setCacheMode(QGraphicsItem::DeviceCoordinateCache);
}

bool ClusterItem::beginVisualTransition(qreal targetOpacity) {
    startOpacity_ = currentOpacity_;
    targetOpacity_ = targetOpacity;
    return std::abs(targetOpacity_ - currentOpacity_) > 0.01;
}

void ClusterItem::applyVisualProgress(qreal progress) {
    const qreal nextOpacity = (progress >= 1.0)
        ? targetOpacity_
        : lerp(startOpacity_, targetOpacity_, progress);
    if (std::abs(nextOpacity - currentOpacity_) < 0.01 && progress < 1.0) {
        return;
    }

    currentOpacity_ = nextOpacity;
    refreshVisualState();
}

QRectF ClusterItem::boundingRect() const {
    return QRectF(-HITBOX_RADIUS, -HITBOX_RADIUS,
                  HITBOX_RADIUS * 2.0, HITBOX_RADIUS * 2.0);
}

QPainterPath ClusterItem::shape() const {
    QPainterPath path;
    path.addEllipse(-HITBOX_RADIUS, -HITBOX_RADIUS,
                    HITBOX_RADIUS * 2.0, HITBOX_RADIUS * 2.0);
    return path;
}

void ClusterItem::paint(QPainter* painter,
                        const QStyleOptionGraphicsItem* option,
                        QWidget* widget) {
    Q_UNUSED(option);
    Q_UNUSED(widget);

    painter->setRenderHint(QPainter::Antialiasing, true);
    painter->setBrush(brush());
    painter->setPen(pen());
    painter->drawEllipse(QRectF(-visualRadius_, -visualRadius_,
                                 visualRadius_ * 2.0, visualRadius_ * 2.0));

    if (memberCount_ > 5) {
        painter->setPen(Qt::white);
        QFont font;
        font.setPixelSize(static_cast<int>(visualRadius_));
        font.setBold(true);
        painter->setFont(font);

        const QString text = (memberCount_ > 999)
            ? QString::number(memberCount_ / 1000) + "k"
            : QString::number(memberCount_);
        const QRectF textRect(-visualRadius_, -visualRadius_,
                              visualRadius_ * 2.0, visualRadius_ * 2.0);
        painter->drawText(textRect, Qt::AlignCenter, text);
    }
}

void ClusterItem::refreshVisualState() {
    const bool visible = currentOpacity_ > 0.02;
    bool needsUpdate = !visualStateInitialized_;

    if (!visualStateInitialized_ || std::abs(lastEffectiveOpacity_ - currentOpacity_) >= 0.01) {
        setOpacity(currentOpacity_);
        lastEffectiveOpacity_ = currentOpacity_;
        needsUpdate = true;
    }

    if (!visualStateInitialized_ || lastVisible_ != visible) {
        setVisible(visible);
        lastVisible_ = visible;
        needsUpdate = true;
    }

    visualStateInitialized_ = true;
    if (needsUpdate) {
        update();
    }
}

} // namespace nav
