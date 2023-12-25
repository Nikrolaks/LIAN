#pragma once

#include "common.h"

#include "gl_const.h"
#include "tinyxml/tinyxml.h"
#include "tinyxml/tinystr.h"


class Map {
public:
    explicit Map(const std::string& fileName);

    bool cellIsTraversable(vec<int64_t> pos) const;
    bool cellOnGrid(vec<int64_t> pos) const;
    bool cellIsObstacle(vec<int64_t> pos) const;

    std::vector<int> &operator[](int64_t i);
    const std::vector<int> &operator[](int64_t i) const;

    int64_t getWidth() const;
    int64_t getHeight() const;
    double getCellSize() const;

    vec<int64_t> getStart() const;
    vec<int64_t> getGoal() const;

private:
    vec<int64_t> start_;
    vec<int64_t> goal_;
    std::vector<std::vector<int>> grid_;
    int64_t height_ = 0, width_ = 0;
    double cellSize_;
};
