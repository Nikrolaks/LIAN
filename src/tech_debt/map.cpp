#include "map.h"


bool Map::cellIsTraversable(vec<int64_t> pos) const {
    return cellOnGrid(pos) && !grid_[pos.y][pos.x];
}

bool Map::cellIsObstacle(vec<int64_t> pos) const {
    return cellOnGrid(pos) && grid_[pos.y][pos.x];
}

bool Map::cellOnGrid(vec<int64_t> pos) const {
    return pos.y >= 0 && pos.y < height_ && pos.x >= 0 && pos.x < width_;
}

std::vector<int>& Map::operator[](int64_t i) {
    return grid_[i];
}

const std::vector<int>& Map::operator[](int64_t i) const {
    return grid_[i];
}

int64_t Map::getHeight() const {
    return height_;
}

int64_t Map::getWidth() const {
    return width_;
}

double Map::getCellSize() const {
    return cellSize_;
}

vec<int64_t> Map::getStart() const {
    return start_;
}

vec<int64_t> Map::getGoal() const {
    return goal_;
}

Map::Map(const std::string& fileName) {
    TiXmlDocument doc(fileName.c_str());
    if (!doc.LoadFile()) {
        throw std::runtime_error("Error openning input XML file");
    }

    TiXmlNode* node = doc.FirstChild();
    bool heightInited = false, widthInited = false;

    while (node) {
        TiXmlElement* element = node->ToElement();
        std::string value = node->Value();

        if (value == CNS_TAG_HEIGHT) {
            height_ = serialize<int64_t>(element);
            heightInited = true;
            grid_.reserve(height_);
        } else if (value == CNS_TAG_WIDTH) {
            width_ = serialize<int64_t>(element);
            widthInited = true;
        } else if (value == CNS_TAG_CELLSIZE) {
            cellSize_ = serialize<double>(element);
            if (cellSize_ <= 0) {
                cellSize_ = 1.f;
            }
        } else if (value == CNS_TAG_SX) {
            start_.x = serialize<int64_t>(element);
            if (start_.x >= width_) {
                throw std::runtime_error((std::stringstream() << "Wrong '" << CNS_TAG_SX << "' value.").str());
            }
        } else if (value == CNS_TAG_SY) {
            start_.y = serialize<int64_t>(element);
            if (start_.y >= height_) {
                throw std::runtime_error((std::stringstream() << "Wrong '" << CNS_TAG_SY << "' value.").str());
            }
        } else if (value == CNS_TAG_FX) {
            goal_.x = serialize<int64_t>(element);
            if (goal_.x >= width_) {
                throw std::runtime_error((std::stringstream() << "Wrong '" << CNS_TAG_FX << "' value.").str());
            }
        } else if (value == CNS_TAG_FY) {
            goal_.y = serialize<int64_t>(element);
            if (goal_.y >= height_) {
                throw std::runtime_error((std::stringstream() << "Wrong '" << CNS_TAG_FY << "' value.").str());
            }
        } else if (value == CNS_TAG_GRID) {
            if (!heightInited || !widthInited) {
                throw std::runtime_error((std::stringstream() << "Error! No '" << CNS_TAG_HEIGHT << "' or '" << CNS_TAG_WIDTH << "' before '" << CNS_TAG_GRID << "' given.").str());
            }

            element = node->FirstChildElement(CNS_TAG_ROW);

            int64_t curY = 0;
            while (curY < height_) {
                if (!element) {
                    throw std::runtime_error((std::stringstream() << "Not enough '" << CNS_TAG_ROW << "' in '" << CNS_TAG_GRID << "' given.").str());
                }

                grid_.push_back(serializeVector<int>(element));
                if (grid_.back().size() != width_) {
                    throw std::runtime_error((std::stringstream() << "Wrong amount of cells in '" << CNS_TAG_ROW << "' " << curY << " given.").str());
                }
                curY++;
                element = element->NextSiblingElement();
            }
        }
        node = doc.IterateChildren(node);
    }
}
