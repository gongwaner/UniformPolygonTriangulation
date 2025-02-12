cmake_minimum_required(VERSION 3.12 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

project(UniformPolygonTriangulation)

find_package(VTK COMPONENTS
        CommonColor
        CommonCore
        CommonDataModel
        CommonComputationalGeometry
        FiltersCore
        FiltersGeneral
        FiltersSources
        FiltersExtraction
        FiltersGeometry
        FiltersModeling
        ImagingHybrid
        IOGeometry
        IOLegacy
        IOPLY
        IOXML
        IOImage
        InteractionStyle
        RenderingContextOpenGL2
        RenderingCore
        RenderingFreeType
        RenderingGL2PSOpenGL2
        RenderingOpenGL2
)

if (NOT VTK_FOUND)
    message(FATAL_ERROR "UniformPolygonTriangulation: Unable to find the VTK build folder.")
endif ()

# Prevent a "command line is too long" failure in Windows.
set(CMAKE_NINJA_FORCE_RESPONSE_FILE "ON" CACHE BOOL "Force Ninja to use response files.")

set(SUBMODULE_SRC_DIR "${CMAKE_SOURCE_DIR}/CommonUtility/src")

add_executable(${PROJECT_NAME} MACOSX_BUNDLE main.cpp
        Algorithm/OptimalPolygonTriangulation.cpp
        Algorithm/SquarePolygonIntersection.cpp
        Algorithm/UniformPolygonTriangulation.cpp
        Utility/ActorUtil.cpp
        Utility/IOUtil.cpp
        Utility/PolygonUtil.cpp
        Utility/TestUtil.cpp
        ${SUBMODULE_SRC_DIR}/Common/Color.h
        ${SUBMODULE_SRC_DIR}/Common/CommonUtil.cpp
        ${SUBMODULE_SRC_DIR}/IO/IOUtil.cpp
        ${SUBMODULE_SRC_DIR}/Mesh/GeometricObjectUtil.cpp
        ${SUBMODULE_SRC_DIR}/Mesh/MeshUtil.cpp
        ${SUBMODULE_SRC_DIR}/Polygon/PolygonUtil.cpp
        ${SUBMODULE_SRC_DIR}/Testing/TestUtil.cpp
        ${SUBMODULE_SRC_DIR}/Transformation/TransformUtil.cpp
)

target_compile_definitions(${PROJECT_NAME}
        PRIVATE
        CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

target_include_directories(${PROJECT_NAME}
        PRIVATE
        ${PROJECT_SOURCE_DIR}
        ${SUBMODULE_SRC_DIR}/Common
        ${SUBMODULE_SRC_DIR}/Mesh
        ${SUBMODULE_SRC_DIR}/Transformation
)

target_link_libraries(${PROJECT_NAME} PRIVATE ${VTK_LIBRARIES})

# vtk_module_autoinit is needed
vtk_module_autoinit(
        TARGETS ${PROJECT_NAME}
        MODULES ${VTK_LIBRARIES}
)
