cmake_minimum_required(VERSION 3.25)
project(playground VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_REQUIRED ON)

include(FetchContent)

FetchContent_Declare(
  sciplot
  GIT_REPOSITORY https://github.com/sciplot/sciplot.git
  GIT_TAG        v0.3.1
  GIT_PROGRESS OFF
  GIT_SHALLOW ON 
)

# set(sciplot_BUILD_TESTS OFF CACHE INTERNAL "")
set(SCIPLOT_BUILD_EXAMPLES OFF CACHE INTERNAL "")
set(SCIPLOT_BUILD_TESTS OFF CACHE INTERNAL "")
set(SCIPLOT_BUILD_DOCS OFF CACHE INTERNAL "")

FetchContent_MakeAvailable(sciplot)

add_executable(playground main.cpp)
target_link_libraries(playground PRIVATE sciplot)
