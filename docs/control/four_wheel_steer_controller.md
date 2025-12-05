# four_wheel_steer_controller_node
4輪ステアリングの足回り制御を行うノード

## トピック
### 購読トピック
| トピック名                | 型                             | 説明                         |
|-------------------------|------------------------------|----------------------------|
| /cmd_vel | geometry_msgs/msg/Twist      | ロボットの速度指令               |

### 配信トピック
| トピック名                | 型                             | 説明                         |
|-------------------------|------------------------------|----------------------------|
| /drive_controller/commands                | std_msgs/msg/Float64MultiArray      | ドライブモーターの制御コマンド               |
| /steer_controller/commands                | std_msgs/msg/Float64MultiArray      | ステアリングモーターの制御コマンド               |
| /odom               | nav_msgs/msg/Odometry      | ロボットのオドメトリ情報               |

