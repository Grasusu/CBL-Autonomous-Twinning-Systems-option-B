#!/usr/bin/env python3
"""Digital entity for the plant-health inspection proof of concept."""

import json
import time
from typing import Dict

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from tb3_pesticide_dt.pesticide_logic import (
    build_zones,
    classify_plant_health,
    is_treatment_needed_status,
    recommendation_for_status,
)
from tb3_pesticide_dt.plant_mission_node import (
    DEFAULT_RESIDUES,
    DEFAULT_STATUSES,
    DEFAULT_ZONE_IDS,
    DEFAULT_ZONE_NAMES,
    DEFAULT_ZONE_X,
    DEFAULT_ZONE_Y,
    DEFAULT_ZONE_YAW,
)


class InspectionTwinNode(Node):
    """Mirrors mission state and simulates a hyperspectral inspection camera."""

    def __init__(self):
        super().__init__("inspection_twin_node")
        self._declare_parameters()
        self.zones = self._load_zones()
        self.zones_by_id = {zone.zone_id: zone for zone in self.zones}

        self.state_topic = str(self.get_parameter("state_topic").value)
        self.physical_state_topic = str(self.get_parameter("physical_state_topic").value)
        self.request_topic = str(self.get_parameter("inspection_request_topic").value)
        self.result_topic = str(self.get_parameter("inspection_result_topic").value)
        self.safety_state_topic = str(self.get_parameter("safety_state_topic").value)
        self.marker_topic = str(self.get_parameter("marker_topic").value)
        self.environment_state_topic = str(self.get_parameter("environment_state_topic").value)
        self.digital_control_topic = str(self.get_parameter("digital_control_topic").value)

        self.mirrored_state: Dict = {}
        self.latest_safety_state: Dict = {}
        self.latest_environment_state: Dict = {}
        self.camera_health_override = None
        self.mirrored_camera_health = None  # health reported by the robot's camera
        self.pending_requests: Dict[int, Dict] = {}
        self.zone_results: Dict[str, Dict] = {}
        self.last_state_publish_at = 0.0

        self.create_subscription(String, self.physical_state_topic, self.on_physical_state, 10)
        self.create_subscription(String, self.request_topic, self.on_inspection_request, 10)
        self.create_subscription(String, self.safety_state_topic, self.on_safety_state, 10)
        self.create_subscription(String, self.environment_state_topic, self.on_environment_state, 10)
        self.create_subscription(String, self.digital_control_topic, self.on_digital_control, 10)

        self.pub_state = self.create_publisher(String, self.state_topic, 10)
        self.pub_result = self.create_publisher(String, self.result_topic, 10)
        self.pub_markers = self.create_publisher(MarkerArray, self.marker_topic, 10)

        self.create_timer(0.20, self.tick)
        self.get_logger().info(
            f"InspectionTwinNode started. Mirroring {self.physical_state_topic}; "
            f"results -> {self.result_topic}"
        )

    def _declare_parameters(self):
        self.declare_parameter("physical_state_topic", "/dt/physical/mission_state")
        self.declare_parameter("state_topic", "/dt/digital/mission_state")
        self.declare_parameter("inspection_request_topic", "/dt/physical/inspection_request")
        self.declare_parameter("inspection_result_topic", "/dt/digital/inspection_result")
        self.declare_parameter("safety_state_topic", "/dt/safety_state")
        self.declare_parameter("marker_topic", "/dt/digital/plant_markers")
        self.declare_parameter("environment_state_topic", "/dt/physical/environment_state")
        self.declare_parameter("digital_control_topic", "/dt/digital/control")

        self.declare_parameter("zone_ids", DEFAULT_ZONE_IDS)
        self.declare_parameter("zone_names", DEFAULT_ZONE_NAMES)
        self.declare_parameter("zone_x", DEFAULT_ZONE_X)
        self.declare_parameter("zone_y", DEFAULT_ZONE_Y)
        self.declare_parameter("zone_yaw", DEFAULT_ZONE_YAW)
        self.declare_parameter("zone_plant_stress_indices", DEFAULT_RESIDUES)
        self.declare_parameter("zone_residue_indices", DEFAULT_RESIDUES)
        self.declare_parameter("zone_expected_statuses", DEFAULT_STATUSES)

        self.declare_parameter("plant_stress_threshold", 0.50)
        self.declare_parameter("processing_delay_s", 1.0)
        self.declare_parameter("camera_health", "healthy")
        self.declare_parameter("healthy_confidence", 0.93)
        self.declare_parameter("degraded_confidence", 0.68)
        self.declare_parameter("frame_id", "odom")

    def _load_zones(self):
        zone_ids = self.get_parameter("zone_ids").value
        plant_stress_indices = self.get_parameter("zone_plant_stress_indices").value
        legacy_indices = self.get_parameter("zone_residue_indices").value
        if len(plant_stress_indices) != len(zone_ids) and len(legacy_indices) == len(zone_ids):
            plant_stress_indices = legacy_indices
        return build_zones(
            zone_ids,
            self.get_parameter("zone_names").value,
            self.get_parameter("zone_x").value,
            self.get_parameter("zone_y").value,
            self.get_parameter("zone_yaw").value,
            plant_stress_indices,
            self.get_parameter("zone_expected_statuses").value,
        )

    def on_physical_state(self, msg: String):
        try:
            self.mirrored_state = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Ignoring malformed physical state: {msg.data}")

    def on_safety_state(self, msg: String):
        try:
            self.latest_safety_state = json.loads(msg.data)
        except json.JSONDecodeError:
            return

    def on_environment_state(self, msg: String):
        try:
            self.latest_environment_state = json.loads(msg.data)
        except json.JSONDecodeError:
            return

    def on_digital_control(self, msg: String):
        try:
            command = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Ignoring malformed digital control command: {msg.data}")
            return

        camera_health = str(command.get("camera_health", command.get("set_camera_health", ""))).lower()
        if camera_health not in {"healthy", "degraded", "failed"}:
            self.get_logger().warn(f"Ignoring unsupported camera_health command: {camera_health}")
            return

        self.camera_health_override = camera_health
        self.get_logger().warn(f"Digital control changed camera_health to {camera_health}")

    def on_inspection_request(self, msg: String):
        try:
            request = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Ignoring malformed inspection request: {msg.data}")
            return

        request_id = int(request.get("request_id", 0))
        if request_id <= 0:
            return
        if request_id in self.pending_requests:
            return

        zone_id = request.get("zone_id")
        if zone_id not in self.zones_by_id:
            self.get_logger().warn(f"Inspection requested for unknown zone: {zone_id}")
            return

        ready_at = time.monotonic() + float(self.get_parameter("processing_delay_s").value)
        request["ready_at"] = ready_at
        self.pending_requests[request_id] = request
        self.get_logger().info(f"Queued hyperspectral plant-health inspection for {zone_id}")

    def tick(self):
        now = time.monotonic()
        ready_ids = [
            request_id
            for request_id, request in self.pending_requests.items()
            if now >= float(request["ready_at"])
        ]
        for request_id in ready_ids:
            request = self.pending_requests.pop(request_id)
            self.publish_inspection_result(request)

        self.publish_state(now)
        self.publish_markers()

    def publish_inspection_result(self, request: Dict):
        zone = self.zones_by_id[request["zone_id"]]
        # The reading comes from the robot's camera (carried in the request); the
        # twin only does the ANALYSIS. Fall back to the zone model if an older
        # mission node sent no measurement.
        camera_health = str(request.get("camera_health") or self.current_camera_health()).lower()
        measured = request.get("measured_plant_stress", None)
        if measured is None and camera_health != "failed":
            measured = zone.plant_stress_index
        self.mirrored_camera_health = camera_health

        if camera_health == "failed" or measured is None:
            status = "SENSOR_FAILED"
            plant_stress_index = None
            disease_level = None
            confidence = 0.0
        else:
            plant_stress_index = measured
            disease_level = plant_stress_index
            threshold = float(self.get_parameter("plant_stress_threshold").value)
            status = classify_plant_health(plant_stress_index, threshold)
            if camera_health == "degraded":
                confidence = float(self.get_parameter("degraded_confidence").value)
            else:
                confidence = float(self.get_parameter("healthy_confidence").value)

        result = {
            "event": "INSPECTION_RESULT",
            "request_id": request.get("request_id"),
            "zone_id": zone.zone_id,
            "zone_name": zone.name,
            "status": status,
            "plant_stress_index": plant_stress_index,
            "disease_level": disease_level,
            "recommendation": recommendation_for_status(status),
            "confidence": confidence,
            "camera_health": camera_health,
            "sensor_model": "simulated_hyperspectral_camera",
            "sensor_target": "plant_stress_and_disease_level",
            "source_entity": "digital_twin",
        }
        self.zone_results[zone.zone_id] = result
        self.pub_result.publish(String(data=json.dumps(result, sort_keys=True)))
        self.get_logger().info(
            f"Published inspection result for {zone.zone_id}: {status} "
            f"recommendation={result['recommendation']} (camera={camera_health})"
        )

    def publish_state(self, now: float):
        if (now - self.last_state_publish_at) < 0.5:
            return
        self.last_state_publish_at = now

        payload = {
            "entity": "digital_twin",
            "mode": self.mirrored_state.get("mode", "WAITING_FOR_PHYSICAL_STATE"),
            "mirrored_zone_id": self.mirrored_state.get("current_zone_id"),
            "mirrored_zone_name": self.mirrored_state.get("current_zone_name"),
            "mirrored_pose": self.mirrored_state.get("pose"),
            # Mirror the robot camera's health continuously from the physical
            # state (immediate), falling back to the last reading / local default.
            "camera_health": (self.mirrored_state.get("digital_camera_health")
                              or self.mirrored_camera_health
                              or self.current_camera_health()),
            "digital_control_topic": self.digital_control_topic,
            "pending_inspections": len(self.pending_requests),
            "latest_result": self.latest_result(),
            "latest_safety_state": self.latest_safety_state,
            "latest_environment_state": self.latest_environment_state,
            "mirrored_behavior_speed_scale": self.mirrored_state.get("behavior_speed_scale"),
        }
        self.pub_state.publish(String(data=json.dumps(payload, sort_keys=True)))

    def current_camera_health(self) -> str:
        if self.camera_health_override is not None:
            return str(self.camera_health_override).lower()
        return str(self.get_parameter("camera_health").value).lower()

    def latest_result(self):
        if not self.zone_results:
            return None
        return list(self.zone_results.values())[-1]

    def publish_markers(self):
        marker_array = MarkerArray()
        frame_id = str(self.get_parameter("frame_id").value)
        stamp = self.get_clock().now().to_msg()

        for i, zone in enumerate(self.zones):
            result = self.zone_results.get(zone.zone_id)
            status = result.get("status") if result else zone.expected_status

            plant = Marker()
            plant.header.frame_id = frame_id
            plant.header.stamp = stamp
            plant.ns = "plant_zones"
            plant.id = i
            plant.type = Marker.CYLINDER
            plant.action = Marker.ADD
            plant.pose.position.x = zone.x
            plant.pose.position.y = zone.y
            plant.pose.position.z = 0.15
            plant.scale.x = 0.22
            plant.scale.y = 0.22
            plant.scale.z = 0.30
            plant.color.a = 0.85
            self.apply_status_color(plant, status)
            marker_array.markers.append(plant)

            label = Marker()
            label.header.frame_id = frame_id
            label.header.stamp = stamp
            label.ns = "plant_labels"
            label.id = i + 1000
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = zone.x
            label.pose.position.y = zone.y
            label.pose.position.z = 0.50
            label.scale.z = 0.12
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = f"{zone.zone_id}: {status}"
            marker_array.markers.append(label)

        self.pub_markers.publish(marker_array)

    @staticmethod
    def apply_status_color(marker: Marker, status: str):
        if is_treatment_needed_status(status):
            marker.color.r = 1.0
            marker.color.g = 0.1
            marker.color.b = 0.05
        elif status == "OK":
            marker.color.r = 0.1
            marker.color.g = 0.85
            marker.color.b = 0.25
        elif status == "SENSOR_FAILED":
            marker.color.r = 0.5
            marker.color.g = 0.5
            marker.color.b = 0.5
        else:
            marker.color.r = 1.0
            marker.color.g = 0.75
            marker.color.b = 0.1


def main(args=None):
    rclpy.init(args=args)
    node = InspectionTwinNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
