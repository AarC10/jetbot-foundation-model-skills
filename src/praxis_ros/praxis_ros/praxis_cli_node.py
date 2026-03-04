#!/usr/bin/env python3

import json
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PraxisCLINode(Node):
    def __init__(self) -> None:
        super().__init__("praxis_cli_node")
        self._pub = self.create_publisher(String, "/praxis/command", 10)
        self.create_subscription(String, "/praxis/result", self._on_result, 10)

        self._stopped = threading.Event()
        self._input_thread = threading.Thread(target=self._input_loop, daemon=True)
        self._input_thread.start()

        self.get_logger().info("Praxis CLI ready. Enter commands, prefix with '!' to force regeneration, or type 'quit'.")

    def _on_result(self, msg: String) -> None:
        data = msg.data.strip()
        if not data:
            return

        try:
            payload = json.loads(data)
        except Exception:
            print(f"\n[result] {data}")
            return

        success = bool(payload.get("success", False))
        exec_id = payload.get("exec_id", "")
        latency_ms = payload.get("latency_ms", "")
        error = payload.get("error", "")
        result = payload.get("result", payload.get("data", {}))

        if success:
            print(f"\n[ok] exec_id={exec_id} latency_ms={latency_ms}")
            print(json.dumps(result, indent=2))
        else:
            print(f"\n[error] {error}")

    def _input_loop(self) -> None:
        while rclpy.ok() and not self._stopped.is_set():
            try:
                line = input("> ").strip()
            except (EOFError, KeyboardInterrupt):
                self._stopped.set()
                self._shutdown()
                return

            if not line:
                continue
            if line.lower() in {"quit", "exit"}:
                self._stopped.set()
                self._shutdown()
                return

            out = String()
            if line.startswith("!"):
                payload = {
                    "request": line[1:].strip(),
                    "force_regen": True,
                }
                out.data = json.dumps(payload)
            else:
                out.data = line

            self._pub.publish(out)

    def _shutdown(self) -> None:
        self.get_logger().info("Shutting down Praxis CLI")
        self.create_timer(0.1, lambda: rclpy.shutdown())


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = PraxisCLINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
