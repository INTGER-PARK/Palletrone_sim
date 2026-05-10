#!/usr/bin/env python3
import os, time, signal, math, threading
import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import mujoco
import mujoco.viewer
from palletrone_interfaces.msg import Input, MobObserverInput, PalletroneState, Wrench
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

PHYSICS_HZ = 400.0
K_THRUST = 0.02
GRAVITY = 9.81
RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0
REACTION_TORQUE_COEFF = 0.02
THRUST_ARROW_SCALE = 0.06
THRUST_ARROW_MAX_LEN = 1.0
THRUST_ARROW_WIDTH = 0.01
ALLOCATOR_INV_SQRT2 = 1.0 / math.sqrt(2.0)
ALLOCATOR_R = 0.148492
ALLOCATOR_L_ARM = math.sqrt(2.0) * ALLOCATOR_R
ALLOCATOR_R_Z = 0.075

DELAY_TIME = 0.0
EXTERNAL_WRENCH_CMD_TIMEOUT = 0.2
SIG_POS=1e-4; SIG_VEL=1e-4; SIG_GYRO=1e-4; SIG_SERVO=1e-5 # noise

def quat_to_rpy(q):
    """
    쿼터니언(Quaternion)을 오일러 각(Roll, Pitch, Yaw)으로 변환하는 함수입니다.
    """
    w,x,y,z = q
    yaw   = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    s     = max(-1.0, min(1.0, 2*(w*y - z*x)))
    pitch = math.asin(s)
    roll  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    return np.array([roll, pitch, yaw], float)

def rpy_to_quat(roll, pitch, yaw):
    """
    오일러 각(Roll, Pitch, Yaw)을 쿼터니언(Quaternion)으로 변환하는 함수입니다.
    """
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
        """
        ROS 2 노드 초기화 및 MuJoCo 물리 엔진 설정, 통신(Pub/Sub) 설정, 
        그리고 시뮬레이션 및 뷰어 스레드를 시작하는 생성자입니다.
        """
        super().__init__('palletrone_plant')

        # MuJoCo 모델 로드
        pkg_share = get_package_share_directory('plant')
        xml_path  = os.path.join(pkg_share, 'xml', 'scene.xml')

        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data  = mujoco.MjData(self.model)
        self.model.opt.timestep = 1.0 / PHYSICS_HZ
        
        # 모델의 주요 Body ID 및 모션 캡쳐(mocap) 초기화
        self.base_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, b"base")
        palm_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, b"hand_palm")
        self.palm_mocap_id = int(self.model.body_mocapid[palm_body_id])
        palm_init_pos = np.array(self.model.body_pos[palm_body_id], dtype=float)
        palm_init_quat = np.array(self.model.body_quat[palm_body_id], dtype=float)
        
        # 부드러운 움직임을 위한 Alpha 파라미터 (저주파 통과 필터 역할)
        self.palm_pos_alpha = float(self.declare_parameter('palm_pos_alpha', 1.0).value)
        self.palm_ang_alpha = float(self.declare_parameter('palm_ang_alpha', 1.0).value)
        # MoB에 넘기는 actuation moment의 기준점입니다.
        # "base": base body origin 기준, "com": MuJoCo가 계산한 현재 base subtree CoM 기준.
        self.actuation_moment_reference = str(
            self.declare_parameter('actuation_moment_reference', 'base').value
        ).lower()
        if self.actuation_moment_reference not in ('base', 'com'):
            self.get_logger().warn(
                f"invalid actuation_moment_reference={self.actuation_moment_reference}, using base"
            )
            self.actuation_moment_reference = 'base'

        # MuJoCo 센서 ID 매핑
        def sid(name): return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name.encode())
        def site_id(name): return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, name.encode())
        self.sid_quat = sid("body_quat")
        self.sid_gyro = sid("body_gyro")
        self.sid_pos  = sid("base_pos")
        self.sid_vel  = sid("base_linvel")
        self.sid_servo_ang = [sid("servo1_angle"), sid("servo2_angle"), sid("servo3_angle"),sid("servo4_angle")]
        self.prop_site_ids = [site_id(f"prop{i}_site") for i in range(1, 5)]
        self.prop_reaction_signs = np.array([1.0, -1.0, 1.0, -1.0], dtype=float)

        self.s_adr = self.model.sensor_adr
        self.s_dim = self.model.sensor_dim

        # 제어 입력 관련 변수 초기화
        self.ctrl = np.zeros(8, dtype=float)
        self.ctrl_lo = np.array(self.model.actuator_ctrlrange[:, 0], dtype=float)
        self.ctrl_hi = np.array(self.model.actuator_ctrlrange[:, 1], dtype=float)
        self.last_sat_log_t = 0.0

        # 제어 입력 지연(Delay) 시뮬레이션용 버퍼 초기화
        self.ctrl_recv = np.zeros(8, dtype=float)
        self._delay_len = max(1, int(DELAY_TIME * PHYSICS_HZ))
        self._delay_buf = np.zeros((self._delay_len,8), dtype=float)
        self._delay_idx = 0

        # 상태 관리를 위한 락(Lock) 및 내부 변수들
        self._lock = threading.Lock()
        self._stop = False
        self.prev_linvel_W = None
        self.prev_gyro_I = None
        self.prev_pub_t    = None
        self.palm_target_pos = palm_init_pos.copy()
        self.palm_target_rpy = quat_to_rpy(palm_init_quat)
        self.palm_pos = self.palm_target_pos.copy()
        self.palm_rpy = self.palm_target_rpy.copy()
        self.external_force_body = np.zeros(3, dtype=float)
        self.external_moment_body = np.zeros(3, dtype=float)
        self.last_external_wrench_cmd_wall_t = None
        self.mob_observer_step = 0
        # ROS 통신: Subscribers
        self.sub_input = self.create_subscription(Input, '/input', self.input_callback, 10)
        self.sub_palm_pose = self.create_subscription(Float64MultiArray, '/palm_pose_cmd', self.palm_pose_callback, 10)
        self.sub_external_wrench = self.create_subscription(Wrench, '/external_wrench_cmd', self.external_wrench_callback, 10)
        
        # ROS 통신: Publishers
        self.pub_state = self.create_publisher(PalletroneState, '/palletrone_state', 10)
        self.pub_palm_pose = self.create_publisher(Float64MultiArray, '/palm_pose_state', 10)
        self.pub_cmd_bldc = self.create_publisher(Float64MultiArray, '/actuator_debug/cmd_bldc', 10)
        self.pub_cmd_servo = self.create_publisher(Float64MultiArray, '/actuator_debug/cmd_servo', 10)
        self.pub_real_bldc = self.create_publisher(Float64MultiArray, '/actuator_debug/real_bldc', 10)
        self.pub_real_servo = self.create_publisher(Float64MultiArray, '/actuator_debug/real_servo', 10)
        self.pub_actuation_wrench_actual = self.create_publisher(Wrench, '/actuation_wrench_actual', 10)
        self.pub_mob_observer_input = self.create_publisher(MobObserverInput, '/mob_observer_input', 10)
        self.pub_body_com = self.create_publisher(Float64MultiArray, '/body_com', 10)
        self.pub_allocation_matrix_debug = self.create_publisher(
            Float64MultiArray, '/allocation_matrix_debug', 10)

        # 뷰어 및 시뮬레이션용 멀티스레딩 시작
        self.viewer_thread = threading.Thread(target=self.viewer_loop, daemon=True); self.viewer_thread.start()
        self.sim_thread    = threading.Thread(target=self.sim_loop,    daemon=True); self.sim_thread.start()

    def sensing_state(self, sid):
        """
        주어진 센서 ID(sid)에 해당하는 MuJoCo 센서 측정값을 추출하여 반환합니다.
        """
        adr = self.s_adr[sid]; dim = self.s_dim[sid]
        return np.array(self.data.sensordata[adr:adr+dim], dtype=float)

    def _noisy(self, x, s):
        """
        실제 센서처럼 보이게 하기 위해 원본 데이터(x)에 표준편차(s)의 가우시안 노이즈를 더합니다.
        """
        return x + np.random.normal(0.0, s, size=x.shape)

    def _delay_step(self):
        """
        제어 입력에 물리적/통신 지연(Delay)을 적용합니다.
        원형 버퍼를 사용하여 이전 시간에 수신된 제어값을 반환합니다.
        """
        i = self._delay_idx
        self._delay_buf[i] = self.ctrl_recv
        self._delay_idx = (i + 1) % self._delay_len
        return self._delay_buf[self._delay_idx]

    def input_callback(self, msg: Input):
        """
        '/input' 토픽을 통해 제어 명령을 수신받아 버퍼에 저장합니다.
        """
        with self._lock:
            self.ctrl_recv = np.asarray(msg.u, dtype=float)

    def palm_pose_callback(self, msg: Float64MultiArray):
        """
        손바닥(Palm)의 목표 위치와 자세(명령)를 수신받아 목표값으로 설정합니다.
        """
        if len(msg.data) < 6:
            return
        with self._lock:
            self.palm_target_pos[:] = np.asarray(msg.data[:3], dtype=float)
            self.palm_target_rpy[:] = np.asarray(msg.data[3:6], dtype=float)

    def external_wrench_callback(self, msg: Wrench):
        """
        외부에서 시스템에 가해지는 외란(힘과 모멘트) 명령을 수신하여 저장합니다.
        """
        with self._lock:
            self.external_force_body[:] = np.asarray(msg.force, dtype=float)
            self.external_moment_body[:] = np.asarray(msg.moment, dtype=float)
            self.last_external_wrench_cmd_wall_t = time.perf_counter()

    def _wrap_angle(self, angle):
        """
        각도를 -π 에서 π 사이의 값으로 정규화(Wrapping)합니다.
        """
        return math.atan2(math.sin(angle), math.cos(angle))

    def _update_palm_pose(self):
        """
        손바닥(Palm)의 현재 위치 및 자세를 목표값(Target)을 향해 부드럽게 보간(스무딩)하여 갱신합니다.
        급격한 Kinematic Jump를 방지하기 위해 alpha 값을 사용합니다.
        """
        self.palm_pos += self.palm_pos_alpha * (self.palm_target_pos - self.palm_pos)
        for i in range(3):
            err = self._wrap_angle(self.palm_target_rpy[i] - self.palm_rpy[i])
            self.palm_rpy[i] = self._wrap_angle(self.palm_rpy[i] + self.palm_ang_alpha * err)

    def _apply_palm_pose(self):
        """
        업데이트된 손바닥(Palm)의 위치와 자세를 MuJoCo의 모션 캡처(mocap) 데이터에 실제로 적용합니다.
        """
        self.data.mocap_pos[self.palm_mocap_id] = self.palm_pos
        self.data.mocap_quat[self.palm_mocap_id] = rpy_to_quat(*self.palm_rpy)

    def _apply_external_wrench(self):
        """
        기체(Body) 기준 좌표계로 들어온 외부 힘과 모멘트(Wrench)를 
        World 좌표계로 변환한 뒤 MuJoCo 물리 엔진의 기체 모델에 적용합니다.
        """
        r_wb = np.asarray(self.data.xmat[self.base_body_id], dtype=float).reshape(3, 3)
        if (
            self.last_external_wrench_cmd_wall_t is not None and
            time.perf_counter() - self.last_external_wrench_cmd_wall_t > EXTERNAL_WRENCH_CMD_TIMEOUT
        ):
            self.external_force_body[:] = 0.0
            self.external_moment_body[:] = 0.0

        force_world = r_wb @ self.external_force_body
        moment_world = r_wb @ self.external_moment_body

        self.data.xfrc_applied[self.base_body_id, :] = 0.0
        self.data.xfrc_applied[self.base_body_id, 0:3] = force_world
        self.data.xfrc_applied[self.base_body_id, 3:6] = moment_world

    def _world_acc_to_body_specific_force(self, acc_W):
        """
        World 좌표계 기준의 가속도를 IMU 센서가 측정하는 
        기체 기준 가속도(Specific Force, 중력의 영향 포함)로 변환합니다.
        """
        # IMU 가속도계의 물리적 출력은 world 선가속도 a_W 자체가 아니라 body 좌표계 specific force f_B이다.
        # MuJoCo world는 z-up이고 중력가속도는 g_W = [0, 0, -g] 이므로,
        #   f_W = a_W - g_W = a_W + [0, 0, g]
        #   f_B = R_BW * f_W = R_WB^T * f_W
        # 로 변환한다. 정지 상태에서는 f_B.z가 대략 +9.81 m/s^2가 된다.
        r_wb = np.asarray(self.data.xmat[self.base_body_id], dtype=float).reshape(3, 3)
        gravity_W = np.array([0.0, 0.0, -GRAVITY], dtype=float)
        specific_force_W = acc_W - gravity_W
        return r_wb.T @ specific_force_W

    def _calc_actuation_wrench_body(self, thrust):
        """
        실제 MuJoCo prop site 위치/방향으로 body-frame actuation wrench를 계산합니다.
        moment 기준점은 actuation_moment_reference 파라미터로 base origin 또는 현재 CoM 중 선택합니다.
        """
        r_wb = np.asarray(self.data.xmat[self.base_body_id], dtype=float).reshape(3, 3)
        base_pos_world = np.asarray(self.data.xpos[self.base_body_id], dtype=float)
        if self.actuation_moment_reference == 'com':
            reference_pos_world = np.asarray(self.data.subtree_com[self.base_body_id], dtype=float)
        else:
            reference_pos_world = base_pos_world

        force_world = np.zeros(3, dtype=float)
        moment_world = np.zeros(3, dtype=float)
        for i, site_id in enumerate(self.prop_site_ids):
            site_rot_world = np.asarray(self.data.site_xmat[site_id], dtype=float).reshape(3, 3)
            site_pos_world = np.asarray(self.data.site_xpos[site_id], dtype=float)
            prop_force_world = site_rot_world @ np.array([0.0, 0.0, thrust[i]], dtype=float)
            reaction_torque_world = site_rot_world @ np.array(
                [0.0, 0.0, self.prop_reaction_signs[i] * REACTION_TORQUE_COEFF * thrust[i]],
                dtype=float,
            )
            r_ref_to_site_world = site_pos_world - reference_pos_world
            force_world += prop_force_world
            moment_world += np.cross(r_ref_to_site_world, prop_force_world) + reaction_torque_world

        force_body = r_wb.T @ force_world
        moment_body = r_wb.T @ moment_world
        return force_body, moment_body

    def _publish_body_com(self):
        """
        base origin 기준 CoM 위치를 body frame으로 publish합니다.
        base_to_com_body 단위: m, body frame B.
        """
        r_wb = np.asarray(self.data.xmat[self.base_body_id], dtype=float).reshape(3, 3)
        base_pos_world = np.asarray(self.data.xpos[self.base_body_id], dtype=float)
        com_world = np.asarray(self.data.subtree_com[self.base_body_id], dtype=float)
        base_to_com_body = r_wb.T @ (com_world - base_pos_world)

        msg = Float64MultiArray()
        msg.data = base_to_com_body.tolist()
        self.pub_body_com.publish(msg)

    def _publish_actuation_wrench_actual(self):
        """
        mj_step() 직후의 실제 actuator force와 prop site geometry를 사용해 MoB known input을 publish합니다.
        """
        real_thrust = np.asarray(self.data.actuator_force[0:4], dtype=float)
        force_body, moment_body = self._calc_actuation_wrench_body(real_thrust)

        msg = Wrench()
        msg.force = force_body.tolist()
        msg.moment = moment_body.tolist()
        self.pub_actuation_wrench_actual.publish(msg)
        return force_body, moment_body

    def _publish_mob_observer_input(self, state_msg, actuation_force_body, actuation_moment_body):
        """
        MoB observer 전용 입력을 하나의 topic으로 묶어서 publish합니다.
        같은 mj_step()에서 얻은 state와 actuation wrench가 한 메시지에 들어가므로
        observer에서 서로 다른 step의 두 topic이 섞이는 문제를 피할 수 있습니다.
        """
        msg = MobObserverInput()
        msg.step = self.mob_observer_step
        msg.sim_time = float(self.data.time)
        msg.pos = state_msg.pos
        msg.vel = state_msg.vel
        msg.rpy = state_msg.rpy
        msg.w_rpy = state_msg.w_rpy
        msg.actuation_force = actuation_force_body.tolist()
        msg.actuation_moment = actuation_moment_body.tolist()
        self.pub_mob_observer_input.publish(msg)

    def _calc_actual_allocation_matrix_body(self):
        """
        MuJoCo site 위치/방향으로 계산한 실제 per-motor thrust allocation matrix입니다.
        각 column은 해당 prop에 1 N thrust가 생길 때 [Mx, My, Mz, Fz]^T 입니다.
        """
        r_wb = np.asarray(self.data.xmat[self.base_body_id], dtype=float).reshape(3, 3)
        base_pos_world = np.asarray(self.data.xpos[self.base_body_id], dtype=float)

        a_actual = np.zeros((4, 4), dtype=float)
        for i, site_id in enumerate(self.prop_site_ids):
            site_rot_world = np.asarray(self.data.site_xmat[site_id], dtype=float).reshape(3, 3)
            site_pos_world = np.asarray(self.data.site_xpos[site_id], dtype=float)

            force_body = r_wb.T @ (site_rot_world @ np.array([0.0, 0.0, 1.0], dtype=float))
            reaction_torque_body = r_wb.T @ (
                site_rot_world @ np.array(
                    [0.0, 0.0, self.prop_reaction_signs[i] * REACTION_TORQUE_COEFF],
                    dtype=float,
                )
            )
            r_ref_to_site_body = r_wb.T @ (site_pos_world - base_pos_world)
            moment_body = np.cross(r_ref_to_site_body, force_body) + reaction_torque_body

            a_actual[:, i] = [
                moment_body[0],
                moment_body[1],
                moment_body[2],
                force_body[2],
            ]
        return a_actual

    def _calc_allocator_a1_matrix(self, servo_rad):
        """
        allocator_controller.cpp의 calc_A1(C2)를 Python으로 그대로 옮긴 matrix입니다.
        Pc_는 현재 allocator 코드와 동일하게 zero로 둡니다.
        """
        s1, s2, s3, s4 = np.sin(servo_rad)
        c1, c2, c3, c4 = np.cos(servo_rad)
        pcx = pcy = pcz = 0.0
        zeta = REACTION_TORQUE_COEFF
        r_z = ALLOCATOR_R_Z
        l_arm = ALLOCATOR_L_ARM
        inv_sqrt2 = ALLOCATOR_INV_SQRT2

        a1 = np.zeros((4, 4), dtype=float)
        a1[0, 0] = inv_sqrt2 * ( zeta + r_z - pcz) * s1 + ( +l_arm / math.sqrt(2.0) - pcy) * c1
        a1[0, 1] = inv_sqrt2 * (-zeta - r_z + pcz) * s2 + ( +l_arm / math.sqrt(2.0) - pcy) * c2
        a1[0, 2] = inv_sqrt2 * (-zeta - r_z + pcz) * s3 + ( -l_arm / math.sqrt(2.0) - pcy) * c3
        a1[0, 3] = inv_sqrt2 * ( zeta + r_z - pcz) * s4 + ( -l_arm / math.sqrt(2.0) - pcy) * c4

        a1[1, 0] = inv_sqrt2 * (-zeta + r_z - pcz) * s1 + ( -l_arm / math.sqrt(2.0) + pcx) * c1
        a1[1, 1] = inv_sqrt2 * (-zeta + r_z - pcz) * s2 + ( +l_arm / math.sqrt(2.0) + pcx) * c2
        a1[1, 2] = inv_sqrt2 * ( zeta - r_z + pcz) * s3 + ( +l_arm / math.sqrt(2.0) + pcx) * c3
        a1[1, 3] = inv_sqrt2 * ( zeta - r_z + pcz) * s4 + ( -l_arm / math.sqrt(2.0) + pcx) * c4

        a1[2, 0] =  zeta * c1
        a1[2, 1] = -zeta * c2
        a1[2, 2] =  zeta * c3
        a1[2, 3] = -zeta * c4

        a1[3, 0] = c1
        a1[3, 1] = c2
        a1[3, 2] = c3
        a1[3, 3] = c4
        return a1

    def _publish_allocation_matrix_debug(self):
        servo_rad = np.array(
            [self.sensing_state(sid)[0] for sid in self.sid_servo_ang],
            dtype=float,
        )
        a_actual = self._calc_actual_allocation_matrix_body()
        a_allocator = self._calc_allocator_a1_matrix(servo_rad)
        a_diff = a_actual - a_allocator

        msg = Float64MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(label="sim_time", size=1, stride=54),
            MultiArrayDimension(label="servo_rad", size=4, stride=53),
            MultiArrayDimension(label="a_actual_row_major_4x4_rows_Mx_My_Mz_Fz", size=16, stride=49),
            MultiArrayDimension(label="a_allocator_row_major_4x4_rows_Mx_My_Mz_Fz", size=16, stride=33),
            MultiArrayDimension(label="a_actual_minus_allocator_row_major_4x4", size=16, stride=17),
            MultiArrayDimension(label="max_abs_diff", size=1, stride=1),
        ]
        msg.data = (
            [float(self.data.time)] +
            servo_rad.tolist() +
            a_actual.reshape(-1).tolist() +
            a_allocator.reshape(-1).tolist() +
            a_diff.reshape(-1).tolist() +
            [float(np.max(np.abs(a_diff)))]
        )
        self.pub_allocation_matrix_debug.publish(msg)

    def _update_thrust_arrows(self, user_scn):
        """
        viewer user scene에 각 prop site의 실제 추력 벡터를 arrow로 표시합니다.
        """
        if len(user_scn.geoms) < len(self.prop_site_ids):
            return

        thrust = np.asarray(self.data.actuator_force[0:4], dtype=float)
        user_scn.ngeom = len(self.prop_site_ids)

        for i, site_id in enumerate(self.prop_site_ids):
            start = np.asarray(self.data.site_xpos[site_id], dtype=float)
            site_rot_world = np.asarray(self.data.site_xmat[site_id], dtype=float).reshape(3, 3)
            direction = site_rot_world @ np.array([0.0, 0.0, 1.0], dtype=float)
            length = np.clip(max(0.0, thrust[i]) * THRUST_ARROW_SCALE, 0.0, THRUST_ARROW_MAX_LEN)
            end = start + direction * length

            geom = user_scn.geoms[i]
            mujoco.mjv_initGeom(
                geom,
                mujoco.mjtGeom.mjGEOM_ARROW,
                np.zeros(3, dtype=float),
                np.zeros(3, dtype=float),
                np.eye(3, dtype=float).reshape(-1),
                np.array([1.0, 0.15, 0.0, 0.85], dtype=np.float32),
            )
            mujoco.mjv_connector(
                geom,
                mujoco.mjtGeom.mjGEOM_ARROW,
                THRUST_ARROW_WIDTH,
                start,
                end,
            )

    def _publish_state(self):
        """
        센서 데이터 수집, 노이즈 추가, 가속도 계산을 수행한 뒤
        기체의 현재 상태(위치, 속도, 가속도, 자세 등)와 손바닥 자세를 ROS 토픽으로 발행합니다.
        """
        quat_imu_W = self.sensing_state(self.sid_quat)
        gyro_I     = self._noisy(self.sensing_state(self.sid_gyro), SIG_GYRO)
        pos_W      = self._noisy(self.sensing_state(self.sid_pos),  SIG_POS)
        linvel_W   = self._noisy(self.sensing_state(self.sid_vel), SIG_VEL)
        servo = self._noisy(np.array([self.sensing_state(sid)[0] for sid in self.sid_servo_ang], dtype=float), SIG_SERVO)
        servo_deg = servo * RAD2DEG
        rpy = quat_to_rpy(quat_imu_W)

        t = self.data.time

        if self.prev_pub_t is None:
            acc_W = np.zeros(3)
            a_rpy = np.zeros(3)
        else:
            dt = max(1e-6, t - self.prev_pub_t)
            acc_W = (linvel_W - self.prev_linvel_W) / dt # 선가속도 미분 계산
            a_rpy = (gyro_I - self.prev_gyro_I) / dt     # 각가속도 미분 계산

        acc_B = self._world_acc_to_body_specific_force(acc_W)

        self.prev_pub_t    = t
        self.prev_linvel_W = linvel_W.copy()
        self.prev_gyro_I = gyro_I.copy()

        # 기체 상태 메시지 구성 및 퍼블리시
        msg = PalletroneState()
        msg.pos   = pos_W.tolist()
        msg.vel   = linvel_W.tolist()
        msg.acc   = acc_B.tolist()
        msg.rpy   = rpy.tolist()
        msg.w_rpy = gyro_I.tolist()
        msg.a_rpy = a_rpy.tolist()
        msg.servo = servo_deg.tolist()

        self.pub_state.publish(msg)

        # 손바닥(Palm) 포즈 메시지 구성 및 퍼블리시
        msg_palm_pose = Float64MultiArray()
        msg_palm_pose.data = np.concatenate((self.palm_pos, self.palm_rpy)).tolist()
        self.pub_palm_pose.publish(msg_palm_pose)
        return msg

    def sim_loop(self):
        """
        메인 물리 시뮬레이션 루프입니다.
        제어 입력을 액추에이터 명령으로 변환하고, 물리 엔진 스텝을 진행하며, 결과를 퍼블리시합니다.
        """
        next_step = time.perf_counter()

        while rclpy.ok() and not self._stop:

            now = time.perf_counter()

            with self._lock:
                self._update_palm_pose()
                self._apply_palm_pose()
                self._apply_external_wrench()
                self.ctrl = self._delay_step()

                # BLDC 모터 추력과 서보 모터 각도로 분리하여 명령어 생성
                cmd = np.empty(8, dtype=float)
                cmd[0:4] = K_THRUST * (self.ctrl[0:4]**2)
                cmd[4:8] = self.ctrl[4:8]
                # 물리적 한계를 넘지 않도록 값 제한(Clipping)
                ctrl_applied = np.clip(cmd, self.ctrl_lo, self.ctrl_hi)
                self.data.ctrl[:] = ctrl_applied

                # 입력 포화(Saturation) 발생 시 로그 출력 (물리적 한계 초과 경고)
                if np.any(np.abs(ctrl_applied - cmd) > 1e-6) and (now - self.last_sat_log_t) > 0.2:
                    self.get_logger().warn(
                        "actuator saturation: "
                        f"cmd_servo_deg={np.round(cmd[4:8] * RAD2DEG, 3).tolist()} "
                        f"ctrl_servo_deg={np.round(ctrl_applied[4:8] * RAD2DEG, 3).tolist()} "
                        f"cmd_bldc={np.round(cmd[0:4], 3).tolist()} "
                        f"ctrl_bldc={np.round(ctrl_applied[0:4], 3).tolist()}"
                    )
                    self.last_sat_log_t = now

                # 물리 엔진 진행 (주기에 맞춰 시뮬레이션 갱신)
                while now >= next_step:
                    mujoco.mj_step(self.model, self.data)
                    self._publish_body_com()
                    actuation_force_body, actuation_moment_body = self._publish_actuation_wrench_actual()
                    self._publish_allocation_matrix_debug()
                    state_msg = self._publish_state()
                    self._publish_mob_observer_input(
                        state_msg,
                        actuation_force_body,
                        actuation_moment_body,
                    )
                    self.mob_observer_step += 1
                    next_step += 1.0 / PHYSICS_HZ

                # 실제 시뮬레이션 상의 모터 상태 확인 및 디버깅용 퍼블리시
                real_servo = np.array([self.sensing_state(sid)[0] for sid in self.sid_servo_ang], dtype=float)
                real_servo_deg = real_servo * RAD2DEG
                real_bldc = np.array(self.data.actuator_force[0:4], dtype=float)

                msg_cmd_bldc = Float64MultiArray()
                msg_cmd_bldc.data = cmd[0:4].tolist()
                self.pub_cmd_bldc.publish(msg_cmd_bldc)

                msg_cmd_servo = Float64MultiArray()
                msg_cmd_servo.data = (cmd[4:8] * RAD2DEG).tolist()
                self.pub_cmd_servo.publish(msg_cmd_servo)

                msg_real_bldc = Float64MultiArray()
                msg_real_bldc.data = real_bldc.tolist()
                self.pub_real_bldc.publish(msg_real_bldc)

                msg_real_servo = Float64MultiArray()
                msg_real_servo.data = real_servo_deg.tolist()
                self.pub_real_servo.publish(msg_real_servo)

            sleep_t = next_step - time.perf_counter()

            if sleep_t > 0:
                time.sleep(sleep_t)

    def viewer_loop(self):
        """
        MuJoCo 3D 뷰어를 실행하여 화면에 시뮬레이션되는 상태를 렌더링하고 동기화하는 루프입니다.
        """
        try:
            viewer_ref = {"handle": None}

            def key_callback(keycode):
                # MuJoCo has geom groups 0..5 only. Use key 6 as a custom shortcut
                # for collision visualization group 5.
                if keycode != ord('6'):
                    return
                viewer_handle = viewer_ref["handle"]
                if viewer_handle is None:
                    return
                with viewer_handle.lock():
                    viewer_handle.opt.geomgroup[5] = 1 - viewer_handle.opt.geomgroup[5]

            with mujoco.viewer.launch_passive(
                self.model,
                self.data,
                key_callback=key_callback,
            ) as viewer:
                viewer_ref["handle"] = viewer
                while viewer.is_running() and rclpy.ok() and not self._stop:
                    with self._lock:
                        self._update_thrust_arrows(viewer.user_scn)
                        viewer.sync()
        except Exception as e:
            self.get_logger().warn(f"viewer end: {e}")

    def close(self):
        """
        노드 종료 시 백그라운드 스레드들이 안전하게 종료될 수 있도록 정지 플래그를 세팅합니다.
        """
        self._stop = True

def main():
    """
    스크립트 진입점입니다. ROS 2 환경을 초기화하고 노드를 실행하며
    인터럽트(Ctrl+C) 발생 시 자원을 안전하게 정리(cleanup)합니다.
    """
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
