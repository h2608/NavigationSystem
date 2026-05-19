#ifndef TRAFFICMODEL_H
#define TRAFFICMODEL_H

namespace nav {

// Stateless traffic cost utilities.
// Travel time uses a bounded BPR-style multiplier:
// T = C * L * (1 + alpha * min(n / v, maxRatio)^beta)
class TrafficModel {
public:
    static constexpr double C = 1.0;
    static constexpr double BPR_ALPHA = 0.15;
    static constexpr double BPR_BETA = 4.0;
    static constexpr double MAX_CONGESTION_RATIO = 3.0;
    static constexpr double VISUAL_YELLOW_RATIO = 0.60;
    static constexpr double VISUAL_RED_RATIO = 0.90;

    static double calculateTravelTime(double length, double capacity, double currentCount);
    static double getCongestionRatio(double capacity, double currentCount);
    static int getCongestionStatus(double capacity, double currentCount);
};

} // namespace nav

#endif // TRAFFICMODEL_H
