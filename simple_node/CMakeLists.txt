cmake_minimum_required(VERSION 3.8)
project(simple_node)

# コンパイラ警告を有効にする
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依存関係を探す
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)


#
# Talkerライブラリの定義
#
add_library(simple_talker_lib SHARED src/simple_talker.cpp)

# amentパッケージへの依存関係を指定
ament_target_dependencies(simple_talker_lib
  rclcpp
  std_msgs
  yaml-cpp
)


# インクルードディレクトリを指定
target_include_directories(simple_talker_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


#
# Talker実行可能ファイルの定義
#
add_executable(simple_talker src/simple_talker_main.cpp)

# amentパッケージへの依存関係を指定
ament_target_dependencies(simple_talker
  rclcpp
  yaml-cpp
)

# 作成したライブラリをリンクする
# (simple_talker_lib が PUBLIC で yaml-cpp をリンクしているため、ここでの追加は不要)
target_link_libraries(simple_talker
  simple_talker_lib
  yaml-cpp
)

# Listenerライブラリの定義
add_library(simple_listener_lib SHARED src/simple_listener.cpp)
# amentパッケージへの依存関係を指定
ament_target_dependencies(simple_listener_lib
  rclcpp
  std_msgs
  yaml-cpp
)
# インクルードディレクトリを指定
target_include_directories(simple_listener_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(simple_listener src/simple_listener_main.cpp)
# amentパッケージへの依存関係を指定
ament_target_dependencies(simple_listener
  rclcpp
  yaml-cpp
)

target_link_libraries(simple_listener
  simple_listener_lib
  yaml-cpp
)

#
# インストール設定
#
install(TARGETS
  simple_talker
  simple_listener
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  simple_talker_lib
  simple_listener_lib
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
