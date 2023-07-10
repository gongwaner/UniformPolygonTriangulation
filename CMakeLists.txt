cmake_minimum_required(VERSION 3.12 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

if (APPLE)
    set(CMAKE_OSX_ARCHITECTURES arm64)
endif ()

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
add_executable(UniformPolygonTriangulation MACOSX_BUNDLE main.cxx
        Algorithm/OptimalPolygonTriangulation.cpp Algorithm/SquarePolygonIntersection.cpp Algorithm/UniformPolygonTriangulation.cpp
        Utility/ActorUtil.cpp Utility/GeometricObjectUtil.cpp Utility/IOUtil.cpp Utility/MeshUtil.cpp Utility/PolygonUtil.cpp Utility/TestUtil.cpp)
target_link_libraries(UniformPolygonTriangulation PRIVATE ${VTK_LIBRARIES})
# vtk_module_autoinit is needed
vtk_module_autoinit(
        TARGETS UniformPolygonTriangulation
        MODULES ${VTK_LIBRARIES}
)