cmake_minimum_required(VERSION 3.27)
project(robots)

set(CMAKE_CXX_STANDARD 23)

file(GLOB helloworld_SRC CONFIGURE_DEPENDS "./source/*.cpp")

add_executable(main main.cpp ${helloworld_SRC})

#add_executable(testVHACD ../source/TestVHACD.cpp)

include_directories("./include")

find_package(unofficial-omniverse-physx-sdk CONFIG REQUIRED)
target_link_libraries(main PRIVATE unofficial::omniverse-physx-sdk::sdk)

find_package(urdfdom CONFIG REQUIRED)
target_link_libraries(main PRIVATE urdfdom::urdf_parser urdfdom::urdfdom_model urdfdom::urdfdom_world urdfdom::urdfdom_sensor)

find_package(glfw3 CONFIG REQUIRED)
target_link_libraries(main PRIVATE glfw)

find_package(OpenGL REQUIRED)
target_link_libraries(main PRIVATE OpenGL::GL)

find_package(assimp CONFIG REQUIRED)
target_link_libraries(main PRIVATE assimp::assimp)
#target_link_libraries(testVHACD PRIVATE assimp::assimp)

find_package(glm CONFIG REQUIRED)
target_link_libraries(main PRIVATE glm::glm)

find_package(glad CONFIG REQUIRED)
target_link_libraries(main PRIVATE glad::glad)

find_package(imgui CONFIG REQUIRED)
target_link_libraries(main PRIVATE imgui::imgui)


# Optional: import the defined target to copy over the GPU acceleration libraries (3rd party provided by NVIDIA)
if(TARGET unofficial::omniverse-physx-sdk::gpu-library)
    if(UNIX)
        # Add rpath setting to find .so libraries on unix based systems
        set_target_properties(main PROPERTIES
                BUILD_WITH_INSTALL_RPATH TRUE
                INSTALL_RPATH "$ORIGIN"
        )
    endif()
    add_custom_command(TARGET main POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:unofficial::omniverse-physx-sdk::gpu-library>
            $<TARGET_FILE_DIR:main>)
    if(WIN32)
        add_custom_command(TARGET main POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:unofficial::omniverse-physx-sdk::gpu-device-library>
                $<TARGET_FILE_DIR:main>)
    endif()
else()
    message(WARNING "GPU acceleration library target not defined - GPU acceleration will NOT be available!")
endif()