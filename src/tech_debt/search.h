#ifndef SEARCH_H
#define SEARCH_H

#include "gl_const.h"
#include "map.h"
#include "searchresult.h"
#include "logger.h"

class Search {

public:
    Search() {}
    virtual ~Search () {}
    virtual SearchResult startSearch(Logger *Log, const Map &map) = 0;

    SearchResult sresult;
};

#endif
