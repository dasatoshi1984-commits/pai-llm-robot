#!/usr/bin/env python3
"""
LLM Commander ノード（PAI最終課題のAI部分）

日本語の自然言語指示（例:「赤い目印まで行って、そのあと左に90度回って」）を
LLM（Gemini API）に渡し、ロボットの行動計画(JSON)を推論させ、
オドメトリを見ながら1ステップずつ実行する。

ベースライン（Qiita記事）からの工夫点:
  1. キーボード操作 → 自然言語での指示に置き換え（AI推論）
  2. 「1m進む」「90度回る」をタイマーではなくオドメトリでフィードバック制御
  3. 目印の座標をLLMに教え、「赤い所まで行って」のような目的地指示を可能に
  4. LLMの出力をチェック（距離・角度の上限、未知の行動の除外）して安全に実行

使い方:
  export GEMINI_API_KEY="(自分のキー)"   ← キーはコードに書かない！
  ros2 run pai_robot llm_commander
"""
import json
import math
import os
import re
import threading
import time
import urllib.error
import urllib.request

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

# ワールドに置いた目印の座標（pai_world.sdf と合わせる）
MARKERS = {
    "red": (3.0, 0.0),
    "blue": (0.0, 3.0),
    "green": (3.0, -3.0),
}

LINEAR_SPEED = 0.5    # [m/s]
ANGULAR_SPEED = 0.6   # [rad/s]
MAX_DISTANCE = 6.0    # 1ステップで進める最大距離 [m]
STOP_BEFORE_M = 1.0   # 目印の手前で止まる距離 [m]（ロボットの車体が大きいため）

SYSTEM_PROMPT = f"""あなたは2次元平面を走る差動二輪ロボットの行動計画AIです。
ユーザーの日本語の指示を、次のJSONだけで返してください（説明文やコードブロックは不要）。

{{"steps": [ ... ], "comment": "日本語で一言（何をするか）"}}

使える step は次の4種類だけです:
- {{"action": "forward", "distance_m": 数値}}    前進（負の値なら後退）。単位はメートル
- {{"action": "turn", "angle_deg": 数値}}        その場で回転。左回り(反時計回り)が正、右回りが負
- {{"action": "goto", "target": "red|blue|green"}} 指定した色の目印の手前まで移動
- {{"action": "wait", "seconds": 数値}}          その場で待つ

ロボットは最初、原点(0,0)で+x方向を向いています。
目印の座標: 赤(red)={MARKERS['red']}, 青(blue)={MARKERS['blue']}, 緑(green)={MARKERS['green']}
「少し」は0.5m程度、「ぐるっと一周」は360度と解釈してください。
指示がロボットの動作と無関係なら steps を空にして comment で理由を伝えてください。
"""


def call_gemini(user_text: str) -> dict:
    """Gemini API(REST)に問い合わせて行動計画のdictを返す。"""
    api_key = os.environ.get("GEMINI_API_KEY")
    if not api_key:
        raise RuntimeError("環境変数 GEMINI_API_KEY が設定されていません")
    model = os.environ.get("GEMINI_MODEL", "gemini-3.8-flash")
    url = f"https://generativelanguage.googleapis.com/v1beta/models/{model}:generateContent"
    body = {
        "systemInstruction": {"parts": [{"text": SYSTEM_PROMPT}]},
        "contents": [{"role": "user", "parts": [{"text": user_text}]}],
        "generationConfig": {"temperature": 0.1, "responseMimeType": "application/json"},
    }
    req = urllib.request.Request(
        url,
        data=json.dumps(body).encode("utf-8"),
        headers={"Content-Type": "application/json", "x-goog-api-key": api_key},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=60) as res:
            data = json.loads(res.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        detail = e.read().decode("utf-8", errors="ignore")[:500]
        raise RuntimeError(f"Gemini APIエラー {e.code}: {detail}") from e

    text = data["candidates"][0]["content"]["parts"][0]["text"]
    # 念のため ```json ... ``` で囲まれていても取り出せるようにする
    m = re.search(r"\{.*\}", text, re.S)
    if not m:
        raise RuntimeError(f"JSONが見つかりません: {text}")
    return json.loads(m.group(0))


def validate_steps(plan: dict) -> list:
    """LLMの出力をチェックして安全な範囲に収める。"""
    safe = []
    for s in plan.get("steps", []):
        a = s.get("action")
        try:
            if a == "forward":
                d = float(s.get("distance_m", 0))
                safe.append({"action": "forward", "distance_m": max(-MAX_DISTANCE, min(MAX_DISTANCE, d))})
            elif a == "turn":
                ang = float(s.get("angle_deg", 0))
                safe.append({"action": "turn", "angle_deg": max(-720.0, min(720.0, ang))})
            elif a == "goto" and s.get("target") in MARKERS:
                safe.append({"action": "goto", "target": s["target"]})
            elif a == "wait":
                safe.append({"action": "wait", "seconds": max(0.0, min(10.0, float(s.get("seconds", 1))))})
        except (TypeError, ValueError):
            pass  # 数値でない値は無視
    return safe


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def norm_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class LlmCommander(Node):
    def __init__(self):
        super().__init__("llm_commander")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.x = self.y = self.yaw = 0.0
        self.have_odom = False

    def on_odom(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.have_odom = True

    # ---------- 低レベル動作 ----------
    def send(self, lin=0.0, ang=0.0):
        t = Twist()
        t.linear.x = float(lin)
        t.angular.z = float(ang)
        self.pub.publish(t)

    def stop(self):
        for _ in range(3):
            self.send(0.0, 0.0)
            time.sleep(0.05)

    def drive(self, distance: float):
        sx, sy = self.x, self.y
        sign = 1.0 if distance >= 0 else -1.0
        timeout = time.time() + abs(distance) / LINEAR_SPEED * 3 + 5
        while rclpy.ok() and time.time() < timeout:
            moved = math.hypot(self.x - sx, self.y - sy)
            remain = abs(distance) - moved
            if remain <= 0.02:
                break
            speed = LINEAR_SPEED if remain > 0.3 else max(0.1, LINEAR_SPEED * remain / 0.3)
            self.send(sign * speed, 0.0)
            time.sleep(0.05)
        self.stop()

    def rotate_to(self, target_yaw: float):
        timeout = time.time() + 30
        while rclpy.ok() and time.time() < timeout:
            err = norm_angle(target_yaw - self.yaw)
            if abs(err) < math.radians(1.5):
                break
            speed = ANGULAR_SPEED if abs(err) > 0.3 else max(0.1, ANGULAR_SPEED * abs(err) / 0.3)
            self.send(0.0, math.copysign(speed, err))
            time.sleep(0.05)
        self.stop()

    def turn(self, angle_deg: float):
        # 360度を超える回転にも対応するため、180度ずつに分けて回す
        remaining = math.radians(angle_deg)
        while abs(remaining) > 1e-3:
            chunk = max(-math.pi * 0.9, min(math.pi * 0.9, remaining))
            self.rotate_to(norm_angle(self.yaw + chunk))
            remaining -= chunk

    def goto(self, target: str):
        tx, ty = MARKERS[target]
        self.rotate_to(math.atan2(ty - self.y, tx - self.x))
        dist = math.hypot(tx - self.x, ty - self.y) - STOP_BEFORE_M
        if dist > 0:
            self.drive(dist)

    # ---------- 計画の実行 ----------
    def execute(self, steps: list):
        for i, s in enumerate(steps, 1):
            self.get_logger().info(f"[{i}/{len(steps)}] 実行中: {s}")
            a = s["action"]
            if a == "forward":
                self.drive(s["distance_m"])
            elif a == "turn":
                self.turn(s["angle_deg"])
            elif a == "goto":
                self.goto(s["target"])
            elif a == "wait":
                time.sleep(s["seconds"])
        self.get_logger().info(f"完了: 現在位置 x={self.x:.2f}, y={self.y:.2f}, 向き={math.degrees(self.yaw):.0f}度")


def main():
    rclpy.init()
    node = LlmCommander()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("オドメトリ(/odom)待ち...")
    for _ in range(100):
        if node.have_odom:
            break
        time.sleep(0.1)
    if not node.have_odom:
        print("※ /odom が来ていません。Gazeboが再生中(▶)か、ブリッジが動いているか確認してください。")

    print("\n=== 自然言語でロボットに指示してください（終了: q） ===")
    print("例: 赤い目印まで行って / 左に90度回って2m進んで / 青い所に行ってから一周して\n")
    try:
        while rclpy.ok():
            text = input("指示> ").strip()
            if text in ("q", "quit", "exit"):
                break
            if not text:
                continue
            try:
                t0 = time.time()
                plan = call_gemini(text)
                steps = validate_steps(plan)
                print(f"AIの解釈 ({time.time() - t0:.1f}秒): {plan.get('comment', '')}")
                print("行動計画:", json.dumps(steps, ensure_ascii=False))
                node.execute(steps)
            except Exception as e:  # noqa: BLE001
                print(f"エラー: {e}")
                node.stop()
    except (KeyboardInterrupt, EOFError):
        pass
    node.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
