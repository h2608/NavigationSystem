#ifndef ROADSTYLE_H
#define ROADSTYLE_H

#include <QColor>

#include "core/graph/Edge.h"

namespace nav {

struct RoadStyle {
    QColor fillColor;
    QColor casingColor;
    double width;
    double casingWidth;
    double zValue;

    static RoadStyle forTier(DisplayTier tier, RoadClass roadClass, double importanceScore) {
        switch (tier) {
            case DisplayTier::Primary:
                return {QColor(246, 246, 242), QColor(112, 122, 132), 3.4, 4.8, 3.0 + importanceScore * 0.5};
            case DisplayTier::Secondary:
                return {QColor(240, 243, 239), QColor(145, 154, 162), 2.4, 3.4, 2.0 + importanceScore * 0.4};
            case DisplayTier::Local:
                return {QColor(235, 239, 234), QColor(172, 180, 170), 1.6, 2.3, 1.1 + importanceScore * 0.3};
            case DisplayTier::Detail:
            default:
                if (roadClass == RoadClass::Arterial) {
                    return {QColor(243, 243, 238), QColor(122, 132, 142), 2.8, 4.0, 2.6};
                }
                return {QColor(231, 236, 231), QColor(184, 191, 182), 1.1, 1.6, 0.6};
        }
    }

    static QColor congestionColor(int status) {
        switch (status) {
            case 1: return QColor(255, 193, 7);
            case 2: return QColor(244, 67, 54);
            default: return QColor(158, 158, 158);
        }
    }

    static double congestionWidthBoost(int status) {
        switch (status) {
            case 1: return 0.5;
            case 2: return 1.0;
            default: return 0.0;
        }
    }
};

} // namespace nav

#endif // ROADSTYLE_H
