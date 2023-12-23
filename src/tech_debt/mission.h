#pragma once

#include "common.h"

#include "config.h"
#include "liansearch.h"
#include "map.h"
#include "search.h"
#include "searchresult.h"
#include "logger.h"

class Mission {
public:
    Mission(const std::string& fName);
    ~Mission();

    bool createLog();
    void createSearch();
    void startSearch();
    void printSearchResultsToConsole();
    void saveSearchResultsToLog();

private:
    Config      config;
    Map         map;

    Search      *search;
    Logger      *logger;

    const std::string fileName;

    SearchResult sr;
};
