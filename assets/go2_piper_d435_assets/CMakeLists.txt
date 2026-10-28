cmake_minimum_required(VERSION 3.8)
project(go2_piper_d435_assets)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf meshes isaac_sim mujoco gazebo licenses
        DESTINATION share/${PROJECT_NAME})
install(FILES model.config README.md asset_manifest.json joint_metadata.json
        DESTINATION share/${PROJECT_NAME})
ament_package()
