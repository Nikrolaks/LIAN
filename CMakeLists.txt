cmake_minimum_required(VERSION 2.8)
project(LianSearch)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX)
	message(STATUS "GCC detected, adding compile flags")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wall -Wextra")
	set(CMAKE_EXE_LINKER_FLAGS "-static -static-libgcc -static-libstdc++")
endif(CMAKE_COMPILER_IS_GNUCXX)

add_subdirectory(contrib)

add_executable(LianSearch
		src/tech_debt/config.cpp
		src/tech_debt/liansearch.cpp
		src/tech_debt/logger.cpp
		src/tech_debt/main.cpp
		src/tech_debt/map.cpp
		src/tech_debt/mission.cpp
		src/tech_debt/openlist.cpp

		src/tech_debt/common.h
		src/tech_debt/config.h
		src/tech_debt/gl_const.h
		src/tech_debt/liansearch.h
		src/tech_debt/logger.h
		src/tech_debt/map.h
		src/tech_debt/mission.h
		src/tech_debt/node.h
		src/tech_debt/openlist.h
		src/tech_debt/search.h
		src/tech_debt/searchresult.h
)

target_link_libraries(LianSearch PUBLIC tinyxml)
target_include_directories(LianSearch PUBLIC contrib)