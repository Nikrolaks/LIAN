#pragma once

#include "common.h"

class Config {
public:
    explicit Config(const std::string& fileName);

    float getParamValue(std::size_t i) const;

    const std::string& getMapFileName() const;

private:
    std::vector<float> searchParams_;
    std::string mapFileName_;
};
