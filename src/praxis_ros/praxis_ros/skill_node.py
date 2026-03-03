import base64
import json
import os
import threading
from typing import Any, Dict, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from praxis.executor import ExecutionContext, SkillExecutor
from praxis.generator import SkillGenerator
from praxis.llm_client import AnthropicClient
from praxis.storage import SkillStorage
from praxis_ros.srv import ExecuteSkill, GenerateSkill


class PraxisSkillNode(Node):
	def __init__(self) -> None:
		super().__init__("praxis_skill_node")

		self.declare_parameter("llm_provider", os.getenv("PRAXIS_LLM_PROVIDER", "anthropic"))
		self.declare_parameter("skill_db_path", os.getenv("PRAXIS_SKILL_DB_PATH", "~/.praxis/skills.db"))
		self.declare_parameter("skill_dir", os.getenv("PRAXIS_SKILL_DIR", "~/.praxis/skills"))
		self.declare_parameter(
			"reuse_threshold",
			float(os.getenv("PRAXIS_REUSE_THRESHOLD", "0.85")),
		)
		self.declare_parameter("contract", os.getenv("PRAXIS_CONTRACT", "ros_python"))

		self.llm_provider = str(self.get_parameter("llm_provider").value)
		self.skill_db_path = os.path.expanduser(str(self.get_parameter("skill_db_path").value))
		self.skill_dir = os.path.expanduser(str(self.get_parameter("skill_dir").value))
		self.reuse_threshold = float(self.get_parameter("reuse_threshold").value)
		self.contract = str(self.get_parameter("contract").value)

		self._cv_bridge = CvBridge()
		self._latest_image: Optional[np.ndarray] = None
		self._latest_motor: Optional[Twist] = None
		self._cache_lock = threading.Lock()

		self._cached_skill_by_name: Dict[str, Any] = {}
		self._cached_skill_by_key: Dict[Tuple[str, str], Any] = {}

		self.storage = SkillStorage(db_path=self.skill_db_path, skill_dir=self.skill_dir)
		if self.llm_provider.lower() != "anthropic":
			self.get_logger().warning(
				f"Unsupported llm_provider '{self.llm_provider}', falling back to AnthropicClient"
			)
		self.llm_client = AnthropicClient()
		self.generator = SkillGenerator(
			storage=self.storage,
			llm_client=self.llm_client,
			reuse_threshold=self.reuse_threshold,
		)
		self.executor = SkillExecutor(storage=self.storage)

		self._result_pub = self.create_publisher(String, "/praxis/result", 10)

		self.create_subscription(String, "/praxis/command", self._on_command, 10)
		self.create_subscription(Image, "/camera/image_rect", self._on_image, 1)
		self.create_subscription(Twist, "/motor_status", self._on_motor_status, 10)

		self._generate_srv = self.create_service(GenerateSkill, "/praxis/generate", self._handle_generate)
		self._execute_srv = self.create_service(ExecuteSkill, "/praxis/execute", self._handle_execute)

		self._generate_client = self.create_client(GenerateSkill, "/praxis/generate")
		self._execute_client = self.create_client(ExecuteSkill, "/praxis/execute")

		self.get_logger().info("PraxisSkillNode started")

	def _on_image(self, msg: Image) -> None:
		try:
			frame = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
			with self._cache_lock:
				self._latest_image = frame
		except Exception as exc:
			self.get_logger().warning(f"Failed to convert image frame: {exc}")

	def _on_motor_status(self, msg: Twist) -> None:
		with self._cache_lock:
			self._latest_motor = msg

	def _parse_command_message(self, raw_data: str) -> Tuple[str, bool]:
		data = raw_data.strip()
		if not data:
			return "", False

		if data.startswith("{"):
			try:
				payload = json.loads(data)
				request = str(payload.get("request", "")).strip()
				force_regen = bool(payload.get("force_regen", False))
				return request, force_regen
			except Exception:
				pass

		if data.startswith("!"):
			return data[1:].strip(), True
		return data, False

	def _on_command(self, msg: String) -> None:
		request, force_regen = self._parse_command_message(msg.data)
		if not request:
			return

		if not self._generate_client.wait_for_service(timeout_sec=0.5):
			self._publish_result({"success": False, "error": "Generate service unavailable"})
			return

		gen_req = GenerateSkill.Request()
		gen_req.request = request
		gen_req.input_hints = []
		gen_req.output_hints = []
		gen_req.reuse_threshold = 0.0 if force_regen else self.reuse_threshold

		gen_future = self._generate_client.call_async(gen_req)

		def on_generate_done(future: Any) -> None:
			try:
				gen_resp = future.result()
				if gen_resp is None or not gen_resp.success:
					self._publish_result(
						{
							"success": False,
							"error": (gen_resp.error if gen_resp is not None else "Unknown generation failure"),
							"request": request,
						}
					)
					return

				params = self._build_context_params(request)
				if not self._execute_client.wait_for_service(timeout_sec=0.5):
					self._publish_result({"success": False, "error": "Execute service unavailable"})
					return

				exec_req = ExecuteSkill.Request()
				exec_req.skill_name = gen_resp.skill_name
				exec_req.version = 0
				exec_req.params_json = json.dumps(params)
				exec_req.user_request = request
				exec_req.timeout_seconds = 0

				exec_future = self._execute_client.call_async(exec_req)

				def on_execute_done(exec_done: Any) -> None:
					try:
						exec_resp = exec_done.result()
						if exec_resp is None:
							self._publish_result(
								{
									"success": False,
									"error": "Execute returned no response",
									"request": request,
								}
							)
							return

						payload = {
							"success": bool(exec_resp.success),
							"request": request,
							"skill_name": gen_resp.skill_name,
							"version_id": gen_resp.version_id,
							"was_reused": bool(gen_resp.was_reused),
							"reuse_similarity": float(gen_resp.reuse_similarity),
							"result": self._safe_json_load(exec_resp.result_json),
							"error": exec_resp.error,
							"exec_id": exec_resp.exec_id,
							"latency_ms": int(exec_resp.latency_ms),
						}
						self._publish_result(payload)
					except Exception as exc:
						self._publish_result({"success": False, "error": f"Execute callback failed: {exc}"})

				exec_future.add_done_callback(on_execute_done)
			except Exception as exc:
				self._publish_result({"success": False, "error": f"Generate callback failed: {exc}"})

		gen_future.add_done_callback(on_generate_done)

	def _build_context_params(self, request: str) -> Dict[str, Any]:
		params: Dict[str, Any] = {"user_request": request}
		with self._cache_lock:
			latest_image = self._latest_image.copy() if self._latest_image is not None else None
			latest_motor = self._latest_motor

		if latest_image is not None:
			ok, png_buffer = cv2.imencode(".png", latest_image)
			if ok:
				params["image_b64"] = base64.b64encode(png_buffer).decode("utf-8")

		if latest_motor is not None:
			params["motor_linear_x"] = float(latest_motor.linear.x)
			params["motor_angular_z"] = float(latest_motor.angular.z)

		return params

	def _publish_result(self, payload: Dict[str, Any]) -> None:
		out = String()
		out.data = json.dumps(payload)
		self._result_pub.publish(out)

	def _safe_json_load(self, text: str) -> Any:
		if not text:
			return {}
		try:
			return json.loads(text)
		except Exception:
			return {"raw": text}

	def _cache_generated_skill(self, gen_result: Any) -> None:
		skill = getattr(gen_result, "skill", None)
		skill_name = getattr(gen_result, "skill_name", "")
		version_id = str(getattr(gen_result, "version_id", ""))
		if skill is None or not skill_name:
			return
		self._cached_skill_by_name[skill_name] = skill
		if version_id:
			self._cached_skill_by_key[(skill_name, version_id)] = skill

	def _resolve_skill(self, skill_name: str, version: int) -> Any:
		if skill_name in self._cached_skill_by_name:
			return self._cached_skill_by_name[skill_name]

		version_id = str(version) if version > 0 else ""
		if version_id and (skill_name, version_id) in self._cached_skill_by_key:
			return self._cached_skill_by_key[(skill_name, version_id)]

		if hasattr(self.storage, "get_skill"):
			if version > 0:
				return self.storage.get_skill(skill_name, version=version)
			return self.storage.get_skill(skill_name)
		if hasattr(self.storage, "load_skill"):
			if version > 0:
				return self.storage.load_skill(skill_name, version=version)
			return self.storage.load_skill(skill_name)
		if hasattr(self.storage, "get"):
			if version > 0:
				return self.storage.get(skill_name, version=version)
			return self.storage.get(skill_name)
		raise RuntimeError(f"Could not resolve skill '{skill_name}'")

	def _handle_generate(self, request: GenerateSkill.Request, response: GenerateSkill.Response) -> GenerateSkill.Response:
		try:
			kwargs: Dict[str, Any] = {"contract_name": self.contract}
			if request.input_hints:
				kwargs["input_hints"] = list(request.input_hints)
			if request.output_hints:
				kwargs["output_hints"] = list(request.output_hints)
			if request.reuse_threshold > 0.0:
				kwargs["reuse_threshold"] = float(request.reuse_threshold)

			try:
				gen_result = self.generator.generate(request.request, **kwargs)
			except TypeError:
				gen_result = self.generator.generate(request.request, contract_name=self.contract)

			self._cache_generated_skill(gen_result)

			response.success = True
			response.version_id = str(getattr(gen_result, "version_id", ""))
			response.skill_name = str(getattr(gen_result, "skill_name", ""))
			response.short_desc = str(getattr(gen_result, "short_desc", ""))
			response.was_reused = bool(getattr(gen_result, "was_reused", False))
			response.reuse_similarity = float(getattr(gen_result, "reuse_similarity", 0.0))
			response.error = ""
		except Exception as exc:
			response.success = False
			response.version_id = ""
			response.skill_name = ""
			response.short_desc = ""
			response.was_reused = False
			response.reuse_similarity = 0.0
			response.error = str(exc)
			self.get_logger().error(f"Generate failed: {exc}")
		return response

	def _handle_execute(self, request: ExecuteSkill.Request, response: ExecuteSkill.Response) -> ExecuteSkill.Response:
		try:
			params = self._safe_json_load(request.params_json)
			if not isinstance(params, dict):
				params = {"input": params}

			context = ExecutionContext(user_request=request.user_request, params=params)
			skill = self._resolve_skill(request.skill_name, int(request.version))
			exec_result = self.executor.execute(skill, context)

			response.success = bool(getattr(exec_result, "success", False))
			response.result_json = json.dumps(getattr(exec_result, "data", {})) if response.success else ""
			response.error = str(getattr(exec_result, "error", ""))
			response.exec_id = str(getattr(exec_result, "exec_id", ""))
			response.latency_ms = int(getattr(exec_result, "latency_ms", 0))
		except Exception as exc:
			response.success = False
			response.result_json = ""
			response.error = str(exc)
			response.exec_id = ""
			response.latency_ms = 0
			self.get_logger().error(f"Execute failed: {exc}")
		return response


def main(args: Optional[list] = None) -> None:
	rclpy.init(args=args)
	node = PraxisSkillNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == "__main__":
	main()
