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

include_directories(
        ${PROJECT_SOURCE_DIR}/Utility
        ${PROJECT_SOURCE_DIR}/CommonUtility
)

add_executable(${PROJECT_NAME} MACOSX_BUNDLE main.cpp
        Algorithm/OptimalPolygonTriangulation.cpp
        Algorithm/SquarePolygonIntersection.cpp
        Algorithm/UniformPolygonTriangulation.cpp
        Utility/ActorUtil.cpp
        Utility/IOUtil.cpp
        Utility/PolygonUtil.cpp
        Utility/TestUtil.cpp
        CommonUtility/Common/CommonUtil.cpp
        CommonUtility/IO/IOUtil.cpp
        CommonUtility/Mesh/GeometricObjectUtil.cpp
        CommonUtility/Mesh/MeshUtil.cpp
        CommonUtility/Polygon/PolygonUtil.cpp
        CommonUtility/Test/TestUtil.cpp
        CommonUtility/Transformation/TransformUtil.cpp
)

target_link_libraries(${PROJECT_NAME} PRIVATE ${VTK_LIBRARIES})

# vtk_module_autoinit is needed
vtk_module_autoinit(
        TARGETS ${PROJECT_NAME}
        MODULES ${VTK_LIBRARIES}
)
