cmake_minimum_required(VERSION 3.8)
project(composable_simple_node)

# コンパイラ警告を有効にする
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依存関係を探す
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(simple_node REQUIRED)


#
# Talkerライブラリの定義
#
add_library(composable_simple_talker SHARED src/composable_simple_talker.cpp)

target_include_directories(composable_simple_talker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(composable_simple_talker PRIVATE c_std_17 cxx_std_17)

# amentパッケージへの依存関係を指定
ament_target_dependencies(composable_simple_talker
  rclcpp
  rclcpp_components
  std_msgs
  yaml-cpp
  simple_node
)

rclcpp_components_register_nodes(composable_simple_talker "composable_simple_node::ComposableSimpleTalker")

#
# Listenerライブラリの定義
#
add_library(composable_simple_listener SHARED src/composable_simple_listener.cpp)

target_include_directories(composable_simple_listener PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(composable_simple_listener PRIVATE c_std_17 cxx_std_17)

# amentパッケージへの依存関係を指定
ament_target_dependencies(composable_simple_listener
  rclcpp
  rclcpp_components
  std_msgs
  yaml-cpp
  simple_node
)

rclcpp_components_register_nodes(composable_simple_listener "composable_simple_node::ComposableSimpleListener")

#
# インストール設定
#
install(TARGETS
  composable_simple_talker
  composable_simple_listener
  EXPORT export_composable_simple_talker
  EXPORT export_composable_simple_listener
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_include_directories(include)
ament_export_libraries(simple_talker_lib simple_listener_lib)
ament_export_dependencies(rclcpp std_msgs yaml-cpp)

# amentパッケージとして登録
ament_package()
