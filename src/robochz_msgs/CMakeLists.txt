cmake_minimum_required(VERSION 3.8)
project(robochz_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── 의존성 찾기 ──────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)   # 인터페이스 코드 생성 매크로 제공
find_package(geometry_msgs REQUIRED)               # PoseStamped 정의 소스
find_package(std_msgs REQUIRED)                    # Header 정의 소스
find_package(action_msgs REQUIRED)                 # Action 빌드에 필요

# ── 인터페이스 코드 생성 ────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PlateDetection.msg"
  "action/RecognizePlate.action"
  DEPENDENCIES geometry_msgs std_msgs action_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 다른 패키지가 우리 인터페이스에 의존할 때, 자동으로 그 패키지의 인터페이스 export까지 잡히도록.
ament_export_dependencies(rosidl_default_runtime)

ament_package()
