# pai_robot — 自然言語で動く差動二輪ロボット（PAI最終課題）

Gazebo上の差動二輪ロボットに、日本語で「赤い目印まで行って」のように指示すると、
LLM（Gemini API）が行動計画(JSON)を推論し、ROS 2 ノードがオドメトリを見ながら実行します。

ベースライン: 「gazeboとROS2を連携してロボットを操作する」(Qiita, N622)
— キーボード操作をLLMによる自然言語操作に置き換え、オドメトリによるフィードバック制御を追加。

## 動作環境
- Ubuntu 22.04 / ROS 2 Humble / Ignition Gazebo Fortress
- `sudo apt install ros-humble-ros-gz ros-humble-teleop-twist-keyboard`

## ビルド
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 実行
```bash
# 端末1: シミュレータ + ブリッジ
ros2 launch pai_robot sim.launch.py
# 端末2: AIコマンダー
export GEMINI_API_KEY="自分のキー"   # キーはコミットしない
ros2 run pai_robot llm_commander
```

## 仕組み
```
自然言語 → [llm_commander] → Gemini API → JSON行動計画 → 安全チェック
        → /cmd_vel (geometry_msgs/Twist) → ros_gz_bridge → Gazebo DiffDrive
Gazebo odometry → ros_gz_bridge → /odom → [llm_commander] (距離・角度のフィードバック)
```
