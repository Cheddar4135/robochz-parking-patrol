# =====================================================
# PlateDetection.msg
#
# 목적
#   번호판 인식 결과를 robochz_monitor에 전달
#
# 필드
#   header       : 인식 완료 시각
#   waypoint_id  : 순찰 지점 ID
#   robot_pose   : 촬영 시점 로봇 위치
#   plate_text   : OCR 결과 번호판
#   confidence   : OCR 신뢰도
#   is_registered: 등록 차량 여부
#   image_path   : 저장 이미지 경로
#   success      : 인식 성공 여부
#   message      : 상태/오류 메시지
#
# 참고
#   header.stamp            : 인식 완료 시각
#   robot_pose.header.stamp : 촬영 요청 시각
#
# 예시
#   header:
#     stamp: 2026-06-20 15:31:22
#     frame_id: "map"
#
#   waypoint_id: "PARKING_A_03"
#
#   robot_pose:
#     header.frame_id: "map"
#     pose.position.x: 3.42
#     pose.position.y: -1.15
#
#   plate_text: "12가3456"
#   confidence: 0.94
#   is_registered: false
#
#   image_path:
#     "/logs/images/20260620_153122.jpg"
#
#   success: true
#   message: ""
# =====================================================

std_msgs/Header header
string waypoint_id
geometry_msgs/PoseStamped robot_pose

string plate_text
float32 confidence
bool is_registered
string image_path

bool success
string message