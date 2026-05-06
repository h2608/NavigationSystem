#ifndef DISPLAYFILTER_H
#define DISPLAYFILTER_H

#include <algorithm>
#include <limits>
#include <vector>

namespace nav {

struct ZoomBand {
    const char* name;
    double enterMinZoom;
    double enterMaxZoom;
    double exitMinZoom;
    double exitMaxZoom;
    double edgeOpacity;
    double minEdgeImportance;
    double widthScale;
    double nodeOpacity;
    double clusterOpacity;
    bool showLabels;
    int clusterLevel;
};

class DisplayFilter {
public:
    DisplayFilter() {
        const double inf = std::numeric_limits<double>::infinity();

        bands_.push_back({
            "overview", 0.0, 0.08, 0.0, 0.08 * 1.12,
            0.92, 0.82, 0.95, 0.0, 0.95, false, 0
        });
        bands_.push_back({
            "city", 0.08, 0.18, 0.08 * 0.88, 0.18 * 1.12,
            0.88, 0.68, 1.00, 0.0, 0.78, false, 0
        });
        bands_.push_back({
            "district", 0.18, 0.35, 0.18 * 0.88, 0.35 * 1.12,
            0.90, 0.55, 1.05, 0.0, 0.42, false, 1
        });
        bands_.push_back({
            "area", 0.35, 0.70, 0.35 * 0.88, 0.70 * 1.12,
            0.94, 0.40, 1.10, 0.18, 0.12, false, 1
        });
        bands_.push_back({
            "street", 0.70, 1.40, 0.70 * 0.88, 1.40 * 1.12,
            0.98, 0.20, 1.18, 0.58, 0.0, true, -1
        });
        bands_.push_back({
            "detail", 1.40, inf, 1.40 * 0.88, inf,
            1.00, 0.0, 1.26, 1.0, 0.0, true, -1
        });
    }

    int getBandIndex(double zoom, int currentIndex) const {
        if (bands_.empty()) return -1;

        const int clampedCurrent = clampIndex(currentIndex);
        if (clampedCurrent >= 0) {
            const ZoomBand& current = bands_[clampedCurrent];
            if (zoom >= current.exitMinZoom && zoom < current.exitMaxZoom) {
                return clampedCurrent;
            }
        }

        for (int i = 0; i < static_cast<int>(bands_.size()); ++i) {
            const ZoomBand& band = bands_[i];
            if (zoom >= band.enterMinZoom && zoom < band.enterMaxZoom) {
                return i;
            }
        }

        if (zoom < bands_.front().enterMaxZoom) return 0;
        return static_cast<int>(bands_.size()) - 1;
    }

    const ZoomBand& getBand(int index) const {
        return bands_[static_cast<size_t>(std::max(0, std::min(index, static_cast<int>(bands_.size()) - 1)))];
    }

private:
    int clampIndex(int index) const {
        if (bands_.empty()) return -1;
        if (index < 0 || index >= static_cast<int>(bands_.size())) return -1;
        return index;
    }

    std::vector<ZoomBand> bands_;
};

} // namespace nav

#endif // DISPLAYFILTER_H
