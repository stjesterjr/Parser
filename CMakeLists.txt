cmake_minimum_required(VERSION 4.2.0)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(App)

add_executable(App)

target_sources(App
PRIVATE
    main.cpp
    parser_callback.cpp
    parser_combined.cpp
    parser_template.cpp
)