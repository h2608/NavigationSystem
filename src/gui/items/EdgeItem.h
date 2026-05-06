#ifndef EDGEITEM_H
#define EDGEITEM_H

#include <QGraphicsLineItem>
#include <QColor>

#include "core/graph/Edge.h"
#include "gui/items/RoadStyle.h"

namespace nav {

class EdgeItem : public QGraphicsLineItem {
public:
    explicit EdgeItem(Edge::Id edgeId, const Point2D& from, const Point2D& to,
                      RoadClass roadClass = RoadClass::Local,
                      DisplayTier displayTier = DisplayTier::Local,
                      double importanceScore = 0.0,
                      QGraphicsItem* parent = nullptr);

    Edge::Id getEdgeId() const { return edgeId_; }
    RoadClass getRoadClass() const { return roadClass_; }

    void updateStyle(int congestionStatus);
    void applyBaseStyle();
    void setSpatialHighlight(bool highlighted, const QColor& color = QColor());
    void setTrafficFocus(bool focused);
    bool beginVisualTransition(qreal targetOpacity, double targetWidthScale);
    void applyVisualProgress(qreal progress);

protected:
    void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget) override;

private:
    void refreshVisualState();
    QColor currentFillColor() const;
    QColor currentCasingColor() const;
    double currentInnerWidth() const;
    double currentOuterWidth() const;

    Edge::Id edgeId_;
    RoadClass roadClass_;
    DisplayTier displayTier_;
    double importanceScore_;
    RoadStyle baseStyle_;
    int congestionStatus_ = 0;
    bool spatialHighlighted_ = false;
    QColor spatialHighlightColor_;
    bool trafficFocused_ = false;

    qreal startOpacity_ = 1.0;
    qreal currentOpacity_ = 1.0;
    qreal targetOpacity_ = 1.0;
    double startWidthScale_ = 1.0;
    double currentWidthScale_ = 1.0;
    double targetWidthScale_ = 1.0;
    bool visualStateInitialized_ = false;
    qreal lastEffectiveOpacity_ = -1.0;
    bool lastVisible_ = true;
    QColor lastBoundingPenColor_;
    double lastBoundingPenWidth_ = -1.0;
    double lastZValue_ = -1.0;
};

} // namespace nav

#endif // EDGEITEM_H
