add_library(example "")

target_sources(
    example
    PRIVATE
        gaussian.cpp
        motionmodel.cpp
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/gaussian.h
        ${CMAKE_CURRENT_LIST_DIR}/motionmodel.h
    )

target_include_directories(
    example
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
    )
