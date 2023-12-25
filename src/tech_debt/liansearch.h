#pragma once

#include "common.h"

#include "gl_const.h"
#include "map.h"
#include "node.h"
#include "openlist.h"
#include "search.h"
#include "logger.h"
#include "config.h"


class LianSearch : public Search {
public:
    LianSearch(SearchParams params);

    ~LianSearch();
    SearchResult startSearch(Logger *Log, const Map &map); // General searching algorithm

private:
    SearchParams params_;

    vec<int64_t> start_, goal_;

    std::vector<int> listOfDistances_;

    int64_t closeSize_; // Number of elements in close (elements that were already examined)

    std::vector<std::vector<circleNode>> circleNodes_; // Virtual nodes that create circle around the cell

    std::vector<std::pair<int,int>> pivotCircle_;  // Vector of nodes (shifts) for pivot security check

    std::vector<float> angles_;

    std::list<Node> lppath_, hppath_; // Final path in two representations
    OpenList open_; // Open : list of nodes waiting for expanding

    std::unordered_multimap<int, Node> close_; // Close: list of nodes that were already expanded

    // Method that calculate Bresenham's Circle (center - (0, 0)) and writing list of created nodes to circleNodes
    void calculateCircle(int radius); // Radius - radius of the circle in cells

    void calculatePivotCircle();

    void calculateDistances();

    void calculateLineSegment(std::vector<Node> &line, const Node &start, const Node &goal); // Method builds Bresenham's Line

    bool checkLineSegment(const Map &map, const Node &start, const Node &goal); // Method builds Bresenham's Line and check it for impassable parts

    // check that there are no obstacle in a safety radius from a turn point
    bool checkPivotCircle(const Map &map, const Node &center);

    double calcAngle(const Node &dad, const Node &node, const Node &son) const;
    bool checkAngle(const Node &dad, const Node &node, const Node &son) const;

    bool stopCriterion(); // Check for the ending criteria. Return true if the algorithm should be stopped

    int tryToIncreaseRadius(Node curNode);
    bool tryToDecreaseRadius(Node &curNode, int width);
    void update(const Node current_node, Node new_node, bool &successors, const Map &map);
    bool expand(const Node curNode, const Map &map);
    std::list<Node> smoothPath(const std::list<Node>& path, const Map& map);
    void makePrimaryPath(Node curNode);
    void makeSecondaryPath();
    double makeAngles();
};
