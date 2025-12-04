# trc2026_ros2
鳥取ローバーチャレンジ2026 ROS2ソフトウェア開発リポジトリ

### 開発環境
- Ubuntu 24.04
- ROS 2 Jazzy

### パッケージ一覧
##### micro_ros_agent : micro-ROS通信インターフェース用パッケージ
##### [trc2026_bringup](bringup/index.md) : ロボット起動用パッケージ
##### [trc2026_control](control/index.md) : ロボット制御用パッケージ    
- [four_wheel_steer_controller_node](control/four_wheel_steer_controller.md) : 4輪ステアリング制御ノード
##### [trc2026_manual](manual/index.md) : 手動コントロール用パッケージ
- [drive_manual_controller_node](manual/drive_manual_controller.md) : 手動コントロールノード
##### [trc2026_navigation](navigation/index.md) : 自律移動用パッケージ
##### [trc2026_description](description/index.md) : ロボットのURDFモデル、RViz表示
##### [trc2026_gazebo](gazebo/index.md) : Gazeboシミュレーション用パッケージ