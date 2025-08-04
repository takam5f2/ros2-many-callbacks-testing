cmake_minimum_required(VERSION 3.8)
project(executors_loader)

# コンパイラ警告を有効にする
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依存関係を探す
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(simple_node REQUIRED)
find_package(composable_simple_node REQUIRED)
find_package(yaml-cpp REQUIRED)



#
# Talker実行可能ファイルの定義
#
add_executable(executors_loader src/executors_loader.cpp)

# amentパッケージへの依存関係を指定
ament_target_dependencies(executors_loader
  rclcpp
  yaml-cpp
  simple_node
  composable_simple_node
)

target_link_libraries(executors_loader
  yaml-cpp
)

#
# インストール設定
#
install(TARGETS
  executors_loader
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)


# amentパッケージとして登録
ament_package()
