#!/usr/bin/env python3
import os, time, signal, math, threading
import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import mujoco
import mujoco.viewer
from palletrone_interfaces.msg import Input, PalletroneState
from std_msgs.msg import Float64MultiArray

PHYSICS_HZ = 400.0
K_THRUST = 0.02
RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0

DELAY_TIME = 0.0
SIG_POS=1e-3; SIG_VEL=1e-3; SIG_GYRO=1e-3; SIG_SERVO=1e-4 # noise

def quat_to_rpy(q):
    w,x,y,z = q
    yaw   = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    s     = max(-1.0, min(1.0, 2*(w*y - z*x)))
    pitch = math.asin(s)
    roll  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    return np.array([roll, pitch, yaw], float)

def rpy_to_quat(roll, pitch, yaw):
    cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return np.array([w, x, y, z], dtype=float)

class PlantRosNode(Node):
    def __init__(self):
        super().__init__('palletrone_plant')

        pkg_share = get_package_share_directory('plant')
        xml_path  = os.path.join(pkg_share, 'xml', 'scene.xml')

        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data  = mujoco.MjData(self.model)
        self.model.opt.timestep = 1.0 / PHYSICS_HZ
        palm_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, b"hand_palm")
        self.palm_mocap_id = int(self.model.body_mocapid[palm_body_id])
        # Alpha < 1 smooths mocap updates to reduce large kinematic jumps of the palm.
        self.palm_pos_alpha = float(self.declare_parameter('palm_pos_alpha', 1.0).value)
        # Alpha < 1 smooths palm orientation changes for the same reason.
        self.palm_ang_alpha = float(self.declare_parameter('palm_ang_alpha', 1.0).value)

        def sid(name): return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name.encode())
        self.sid_quat = sid("body_quat")
        self.sid_gyro = sid("body_gyro")
        self.sid_pos  = sid("base_pos")
        self.sid_vel  = sid("base_linvel")
        self.sid_servo_ang = [sid("servo1_angle"), sid("servo2_angle"), sid("servo3_angle"),sid("servo4_angle")]

        self.s_adr = self.model.sensor_adr
        self.s_dim = self.model.sensor_dim

        self.ctrl = np.zeros(8, dtype=float)
        self.ctrl_lo = np.array(self.model.actuator_ctrlrange[:, 0], dtype=float)
        self.ctrl_hi = np.array(self.model.actuator_ctrlrange[:, 1], dtype=float)
        self.last_sat_log_t = 0.0

        self.ctrl_recv = np.zeros(8, dtype=float)
        self._delay_len = max(1, int(DELAY_TIME * PHYSICS_HZ))
        self._delay_buf = np.zeros((self._delay_len,8), dtype=float)
        self._delay_idx = 0

        self._lock = threading.Lock()
        self._stop = False
        self.prev_linvel_W = None
        self.prev_gyro_I = None
        self.prev_pub_t    = None
        self.palm_target_pos = np.array([0.55, 0.0, 0.5], dtype=float)
        self.palm_target_rpy = np.zeros(3, dtype=float)
        self.palm_pos = self.palm_target_pos.copy()
        self.palm_rpy = self.palm_target_rpy.copy()

        self.sub_input = self.create_subscription(Input, '/input', self.input_callback, 10)
        self.sub_palm_pose = self.create_subscription(Float64MultiArray, '/palm_pose_cmd', self.palm_pose_callback, 10)
        self.pub_state = self.create_publisher(PalletroneState, '/palletrone_state', 10)
        self.pub_actuator_servo_qpos = self.create_publisher(Float64MultiArray, '/actuator_debug/servo_qpos', 10)
        self.pub_requested_bldc = self.create_publisher(Float64MultiArray, '/actuator_debug/requested_bldc', 10)
        self.pub_requested_servo = self.create_publisher(Float64MultiArray, '/actuator_debug/requested_servo', 10)
        self.pub_applied_bldc = self.create_publisher(Float64MultiArray, '/actuator_debug/applied_bldc', 10)
        self.pub_applied_servo = self.create_publisher(Float64MultiArray, '/actuator_debug/applied_servo', 10)

        self.viewer_thread = threading.Thread(target=self.viewer_loop, daemon=True); self.viewer_thread.start()
        self.sim_thread    = threading.Thread(target=self.sim_loop,    daemon=True); self.sim_thread.start()

    def sensing_state(self, sid):
        adr = self.s_adr[sid]; dim = self.s_dim[sid]
        return np.array(self.data.sensordata[adr:adr+dim], dtype=float)

    def _noisy(self, x, s):
        return x + np.random.normal(0.0, s, size=x.shape)

    def _delay_step(self):
        i = self._delay_idx
        self._delay_buf[i] = self.ctrl_recv
        self._delay_idx = (i + 1) % self._delay_len
        return self._delay_buf[self._delay_idx]

    def input_callback(self, msg: Input):
        with self._lock:
            self.ctrl_recv = np.asarray(msg.u, dtype=float)

    def palm_pose_callback(self, msg: Float64MultiArray):
        if len(msg.data) < 6:
            return
        with self._lock:
            self.palm_target_pos[:] = np.asarray(msg.data[:3], dtype=float)
            self.palm_target_rpy[:] = np.asarray(msg.data[3:6], dtype=float)

    def _wrap_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _update_palm_pose(self):
        self.palm_pos += self.palm_pos_alpha * (self.palm_target_pos - self.palm_pos)
        for i in range(3):
            err = self._wrap_angle(self.palm_target_rpy[i] - self.palm_rpy[i])
            self.palm_rpy[i] = self._wrap_angle(self.palm_rpy[i] + self.palm_ang_alpha * err)

    def _apply_palm_pose(self):
        self.data.mocap_pos[self.palm_mocap_id] = self.palm_pos
        self.data.mocap_quat[self.palm_mocap_id] = rpy_to_quat(*self.palm_rpy)

    def sim_loop(self):
        next_step = time.perf_counter()
        next_pub  = next_step

        while rclpy.ok() and not self._stop:

            now = time.perf_counter()

            with self._lock:
                self._update_palm_pose()
                self._apply_palm_pose()
                self.ctrl = self._delay_step()

                requested = np.empty(8, dtype=float)
                requested[0:4] = K_THRUST * (self.ctrl[0:4]**2)
                requested[4:8] = self.ctrl[4:8]
                applied = np.clip(requested, self.ctrl_lo, self.ctrl_hi)
                self.data.ctrl[:] = applied
                servo_qpos = np.array([self.sensing_state(sid)[0] for sid in self.sid_servo_ang], dtype=float)

                servo_qpos_deg = servo_qpos * RAD2DEG

                msg_servo_qpos = Float64MultiArray()
                msg_servo_qpos.data = servo_qpos_deg.tolist()
                self.pub_actuator_servo_qpos.publish(msg_servo_qpos)

                msg_requested_bldc = Float64MultiArray()
                msg_requested_bldc.data = requested[0:4].tolist()
                self.pub_requested_bldc.publish(msg_requested_bldc)

                requested_servo_deg = requested[4:8] * RAD2DEG
                msg_requested_servo = Float64MultiArray()
                msg_requested_servo.data = requested_servo_deg.tolist()
                self.pub_requested_servo.publish(msg_requested_servo)

                msg_applied_bldc = Float64MultiArray()
                msg_applied_bldc.data = applied[0:4].tolist()
                self.pub_applied_bldc.publish(msg_applied_bldc)

                applied_servo_deg = applied[4:8] * RAD2DEG
                msg_applied_servo = Float64MultiArray()
                msg_applied_servo.data = applied_servo_deg.tolist()
                self.pub_applied_servo.publish(msg_applied_servo)

                if np.any(np.abs(applied - requested) > 1e-6) and (now - self.last_sat_log_t) > 0.2:
                    self.get_logger().warn(
                        "actuator saturation: "
                        f"req_servo_deg={np.round(requested_servo_deg, 3).tolist()} "
                        f"applied_servo_deg={np.round(applied_servo_deg, 3).tolist()} "
                        f"servo_qpos_deg={np.round(servo_qpos_deg, 3).tolist()} "
                        f"req_bldc={np.round(requested[0:4], 3).tolist()} "
                        f"applied_bldc={np.round(applied[0:4], 3).tolist()}"
                    )
                    self.last_sat_log_t = now

                while now >= next_step:
                    mujoco.mj_step(self.model, self.data)
                    next_step += 1.0 / PHYSICS_HZ

                while now >= next_pub:
                    quat_imu_W = self.sensing_state(self.sid_quat)
                    gyro_I     = self._noisy(self.sensing_state(self.sid_gyro), SIG_GYRO)
                    pos_W      = self._noisy(self.sensing_state(self.sid_pos),  SIG_POS)
                    linvel_W   = self._noisy(self.sensing_state(self.sid_vel), SIG_VEL)
                    servo = self._noisy(np.array([self.sensing_state(sid)[0] for sid in self.sid_servo_ang], dtype=float), SIG_SERVO)
                    servo_deg = servo * RAD2DEG
                    rpy = quat_to_rpy(quat_imu_W)

                    # 수정된 부분: Wall-clock time 대신 MuJoCo의 시뮬레이션 내부 시간을 사용
                    t = self.data.time

                    if self.prev_pub_t is None:
                        acc_W = np.zeros(3)
                        a_rpy = np.zeros(3)
                    else:
                        dt = max(1e-6, t - self.prev_pub_t)
                        acc_W = (linvel_W - self.prev_linvel_W) / dt
                        a_rpy = (gyro_I - self.prev_gyro_I) / dt

                    self.prev_pub_t    = t
                    self.prev_linvel_W = linvel_W.copy()
                    self.prev_gyro_I = gyro_I.copy()

                    msg = PalletroneState()
                    msg.pos   = pos_W.tolist()
                    msg.vel   = linvel_W.tolist()
                    msg.acc   = acc_W.tolist()
                    msg.rpy   = rpy.tolist()
                    msg.w_rpy = gyro_I.tolist()
                    msg.a_rpy = a_rpy.tolist()
                    msg.servo = servo_deg.tolist()
                    
                    self.pub_state.publish(msg)

                    next_pub += 1.0 / PHYSICS_HZ

            sleep_t = next_step - time.perf_counter()

            if sleep_t > 0:
                time.sleep(sleep_t)

    def viewer_loop(self):
        try:
            with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
                with self._lock:
                    viewer.opt.geomgroup[3] = 0
                while viewer.is_running() and rclpy.ok() and not self._stop:
                    with self._lock:
                        viewer.sync()
        except Exception as e:
            self.get_logger().warn(f"viewer end: {e}")

    def close(self):
        self._stop = True

def main():
    rclpy.init()
    node = PlantRosNode()
    signal.signal(signal.SIGINT, lambda *_: node.close())
    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
