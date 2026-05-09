import asyncio
import rclpy
from rclpy.node import Node
from actuator_msgs.msg import Actuators
from sensor_msgs.msg import NavSatFix, LaserScan
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw
import threading
import math
import time

UAV_PORT = 14581
ROVER_TOPIC = "/model/rover_differential_2/command/motor_speed"
ROVER_NAVSAT = "/world/default/model/rover_differential_2/link/base_link/sensor/navsat_sensor/navsat"
DRONE_NAVSAT = "/world/default/model/x500_lidar_front_1/link/base_link/sensor/navsat_sensor/navsat"
DRONE_LIDAR = "/world/default/model/x500_lidar_front_1/link/lidar_sensor_link/sensor/lidar/scan"
ROVER_LIDAR = "/world/default/model/rover_differential_2/link/lidar_link/sensor/lidar_2d_v2/scan"

ROVER_SPEED = 50.0
TAKEOFF_ALT = 20.0
CLEAR_TIME = 5.0
DRONE_OBS_DIST = 8.0
ROVER_CLEAR_DIST = 10.0
DEBOUNCE_COUNT = 3


class BridgeNode(Node):
    def __init__(self):
        super().__init__('mirror_bridge')

        self.motor_pub = self.create_publisher(Actuators, ROVER_TOPIC, 10)

        self.rover_lat = None
        self.rover_lon = None
        self.rover_alt = None
        self.rover_base_alt = None

        self.ref_lat = None
        self.ref_lon = None

        self.rover_start_lat = None
        self.rover_start_lon = None

        self.drone_lidar_range = float('inf')
        self.drone_obstacle = False
        self._drone_clear_count = 0

        self.rover_scan = []
        self._rover_min_range = float('inf')

        self.rover_safe = False
        self._safe_count = 0

        self.create_subscription(NavSatFix, ROVER_NAVSAT, self._rover_cb, 10)
        self.create_subscription(NavSatFix, DRONE_NAVSAT, self._drone_cb, 10)
        self.create_subscription(LaserScan, DRONE_LIDAR, self._drone_lidar_cb, 10)
        self.create_subscription(LaserScan, ROVER_LIDAR, self._rover_lidar_cb, 10)

    def _drone_cb(self, msg):
        if self.ref_lat is None:
            self.ref_lat = msg.latitude
            self.ref_lon = msg.longitude

    def _rover_cb(self, msg):
        self.rover_lat = msg.latitude
        self.rover_lon = msg.longitude
        self.rover_alt = msg.altitude

    def is_rover_elevated(self):
        if self.rover_alt is None or self.rover_base_alt is None:
            return False
        return (self.rover_alt - self.rover_base_alt) > 0.01

    def _drone_lidar_cb(self, msg):
        if not msg.ranges:
            return

        r = msg.ranges[0]

        if math.isnan(r) or math.isinf(r):
            return

        self.drone_lidar_range = r

        if r < DRONE_OBS_DIST:
            self._drone_clear_count = 0
            self.drone_obstacle = True
        else:
            self._drone_clear_count += 1
            if self._drone_clear_count >= DEBOUNCE_COUNT:
                self.drone_obstacle = False

    def _rover_lidar_cb(self, msg):
        if not msg.ranges:
            return

        self.rover_scan = msg.ranges

        valid = [
            r for r in msg.ranges
            if not math.isnan(r) and not math.isinf(r) and r > 0.1
        ]

        if not valid:
            return

        min_r = min(valid)
        self._rover_min_range = min_r

        if min_r >= ROVER_CLEAR_DIST:
            self._safe_count += 1
            if self._safe_count >= DEBOUNCE_COUNT:
                self.rover_safe = True
        else:
            self._safe_count = 0
            self.rover_safe = False

    def lock_rover_start(self):
        self.rover_start_lat = self.rover_lat
        self.rover_start_lon = self.rover_lon

    def get_rover_delta_ned(self):
        if None in (self.rover_lat, self.rover_start_lat):
            return None, None

        dn = math.radians(self.rover_lat - self.rover_start_lat) * 6371000.0
        de = math.radians(self.rover_lon - self.rover_start_lon) * 6371000.0 * math.cos(math.radians(self.rover_start_lat))

        return dn, de

    def send_motors(self, v1, v2):
        msg = Actuators()
        msg.velocity = [float(v1), float(v2)]
        self.motor_pub.publish(msg)


def ros_spin(node, stop_event):
    while not stop_event.is_set():
        rclpy.spin_once(node, timeout_sec=0.02)


def rover_drive(node, launch_event, stop_event):
    print("[ROVER] Starting")
    print("[ROVER] Waiting for signal")

    while not stop_event.is_set():
        node.send_motors(ROVER_SPEED, ROVER_SPEED)

        if node.rover_lat is not None and node.rover_alt is not None:
            break

        time.sleep(0.05)

    print("[ROVER] Checking ground")

    alt_samples = []
    cal_start = time.time()

    while time.time() - cal_start < 3.0:
        node.send_motors(ROVER_SPEED, ROVER_SPEED)

        if node.rover_alt is not None:
            alt_samples.append(node.rover_alt)

        time.sleep(0.05)

    node.rover_base_alt = min(alt_samples)
    node.lock_rover_start()

    print("[ROVER] Moving forward")

    while not stop_event.is_set():
        node.send_motors(ROVER_SPEED, ROVER_SPEED)

        dn, de = node.get_rover_delta_ned()

        if dn is not None:
            dist = math.hypot(dn, de)
        
            print("[ROVER] Searching for safe ground")

            if dist >= 40.0:
                break

        time.sleep(0.05)

    clear_since = None
    flat_printed = False
    mound_printed = False

    while not stop_event.is_set():
        now = time.time()
        node.send_motors(ROVER_SPEED, ROVER_SPEED)

        alt_diff = (node.rover_alt - node.rover_base_alt) if node.rover_alt else 0
        elevated = alt_diff > 0.01

        if not elevated:
            if clear_since is None:
                clear_since = now

                if not flat_printed:
                    print("[ROVER] Flat ground detected")
                    flat_printed = True

            if now - clear_since >= CLEAR_TIME:
                print("[ROVER] Safe landing area found")
                node.send_motors(0, 0)
                launch_event.set()
                break

        else:
            clear_since = None

            if not mound_printed:
                print("[ROVER] Uneven ground detected")
                mound_printed = True

        time.sleep(0.05)

    print("[ROVER] Waiting for drone")

    while not stop_event.is_set():
        node.send_motors(0, 0)
        time.sleep(0.1)

    print("[ROVER] Done")


async def run_drone(node, launch_event, stop_event):
    def rover_offset_ned():
        dn, de = node.get_rover_delta_ned()
        if dn is None:
            return 0.0, 0.0
        return dn, de

    drone = System()
    await drone.connect(system_address=f"udpout://127.0.0.1:{UAV_PORT}")

    print("[DRONE] Connecting")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("[DRONE] Connected")
            break

    print("[DRONE] Waiting for GPS")

    while node.ref_lat is None or node.rover_lat is None:
        await asyncio.sleep(0.1)

    print("[DRONE] GPS ready")

    try:
        await drone.param.set_param_int("NAV_DLL_ACT", 0)
        await drone.param.set_param_int("COM_RCL_EXCEPT", 7)
    except Exception:
        print("[DRONE] Setup warning ignored")

    await asyncio.sleep(1)

    print("[DRONE] Waiting until ready")

    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("[DRONE] Ready")
            break

        await asyncio.sleep(0.5)

    print("[DRONE] Waiting for rover")

    while not launch_event.is_set():
        await asyncio.sleep(0.5)

    print("[DRONE] Rover found safe area")
    print("[DRONE] Preparing for takeoff")

    await drone.action.arm()
    await asyncio.sleep(1)

    for _ in range(30):
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -TAKEOFF_ALT, 0.0))
        await asyncio.sleep(0.1)

    started = False

    for _ in range(5):
        try:
            await drone.offboard.start()
            started = True
            break
        except OffboardError:
            for _ in range(20):
                await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -TAKEOFF_ALT, 0.0))
                await asyncio.sleep(0.1)

    if not started:
        print("[DRONE] Could not start")
        stop_event.set()
        return

    print("[DRONE] Taking off")

    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -TAKEOFF_ALT, 0.0))
    await asyncio.sleep(15)

    dn, de = rover_offset_ned()

    print("[DRONE] Flying to rover")

    await drone.offboard.set_position_ned(PositionNedYaw(dn, de, -TAKEOFF_ALT, 0.0))
    await asyncio.sleep(8)

    dn, de = rover_offset_ned()

    print("[DRONE] Searching for landing area")

    flat_min = TAKEOFF_ALT - 1.0

    offsets = [
        (0, 3),
        (0, -3),
        (3, 0),
        (-3, 0),
        (0, 5),
        (0, -5),
        (5, 0),
        (-5, 0)
    ]

    land_n, land_e = dn + 3, de

    for on, oe in offsets:
        tn, te = dn + on, de + oe

        await drone.offboard.set_position_ned(PositionNedYaw(tn, te, -TAKEOFF_ALT, 0.0))
        await asyncio.sleep(2)

        lidar = node.drone_lidar_range

        if lidar > flat_min:
            land_n, land_e = tn, te
            break

    print("[DRONE] Landing")

    await drone.offboard.set_position_ned(PositionNedYaw(land_n, land_e, -TAKEOFF_ALT, 0.0))
    await asyncio.sleep(3)

    await drone.action.land()

    print("[DRONE] Mission complete")

    stop_event.set()


def main():
    rclpy.init()
    node = BridgeNode()

    launch_event = threading.Event()
    stop_event = threading.Event()
    ros_stop = threading.Event()
    rt = threading.Thread(target=ros_spin, args=(node, ros_stop), daemon=True)
    rt.start()
    rover_t = threading.Thread(target=rover_drive, args=(node, launch_event, stop_event))
    rover_t.start()

    asyncio.run(run_drone(node, launch_event, stop_event))
    rover_t.join()
    ros_stop.set()
    rclpy.shutdown()

    print("Done")


main()