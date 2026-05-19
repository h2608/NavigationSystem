#include "core/traffic/TrafficModel.h"

#include <algorithm>
#include <cmath>

namespace nav {

double TrafficModel::calculateTravelTime(double length, double capacity, double currentCount) {
    if (capacity <= 0.0) {
        return length * C;
    }

    const double ratio = std::clamp(currentCount / capacity, 0.0, MAX_CONGESTION_RATIO);
    const double multiplier = 1.0 + BPR_ALPHA * std::pow(ratio, BPR_BETA);
    return C * length * multiplier;
}

double TrafficModel::getCongestionRatio(double capacity, double currentCount) {
    if (capacity <= 0.0) {
        return 0.0;
    }
    return std::max(0.0, currentCount / capacity);
}

int TrafficModel::getCongestionStatus(double capacity, double currentCount) {
    const double ratio = getCongestionRatio(capacity, currentCount);

    if (ratio < VISUAL_YELLOW_RATIO) {
        return 0;
    }
    if (ratio < VISUAL_RED_RATIO) {
        return 1;
    }
    return 2;
}

} // namespace nav
