#pragma once

#include "common.h"

#include "gl_const.h"
#include "tinyxml/tinyxml.h"
#include "tinyxml/tinystr.h"


class Map {
public:
    explicit Map(const std::string& fileName);

    bool cellIsTraversable(vec<std::size_t> pos) const;
    bool cellOnGrid(vec<std::size_t> pos) const;
    bool cellIsObstacle(vec<std::size_t> pos) const;

    std::vector<int> &operator[](std::size_t i);
    const std::vector<int> &operator[](std::size_t i) const;

    std::size_t getWidth() const;
    std::size_t getHeight() const;
    double getCellSize() const;

    vec<std::size_t> getStart() const;
    vec<std::size_t> getGoal() const;

private:
    vec<std::size_t> start_;
    vec<std::size_t> goal_;
    std::vector<std::vector<int>> grid_;
    std::size_t height_ = 0, width_ = 0;
    double cellSize_;
};
