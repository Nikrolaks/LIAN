#pragma once

#include "common.h"

struct SearchParams {
    float logLevel;
    double angleLimit;
    double baseSegmentLength;
    double heuristicWeight;
    double curvatureHeuristicWeight;
    std::optional<uint64_t> stepsLimit;
    double distanceDecreaseCoefficient;
    double minSegmentLength;
    double pivotCircleRadius;
    uint64_t parentsToIncreaseRadius;
    bool doSmoothing;
    bool doELian;
};

class Config {
public:
    explicit Config(const std::string& fileName);

    const SearchParams& params() const;

    const std::string& getMapFileName() const;

private:
    SearchParams params_;
    std::string mapFileName_;
};
