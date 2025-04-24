from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

from isaaclab.tasks.base.rl_task_cfg import BaseRLEnvCfg
from isaaclab.assets.unitree.g1_with_hand_cfg import G1WithHandCfg  # ← 우리가 만든 로봇 config

@configclass
class G1GraspEnvCfg(BaseRLEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        # 사용할 로봇 설정
        self.scene.robot = G1WithHandCfg

        # 물체 설정: 잡을 object 하나 생성
        from isaaclab.objects import DynamicCuboidCfg
        self.scene.add(DynamicCuboidCfg(name="pick_cube"))

        # 환경 초기화 수: 하나의 큐브만 있는 간단한 설정
        self.scene.num_envs = 32
        self.scene.env_spacing = 2.5
        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None

        # 보상 구성 (간단한 버전)
        self.rewards.reach_cube = {
            "weight": 1.0,
            "distance_threshold": 0.05,
            "entity_a": SceneEntityCfg("robot", body_names=["left_thumb_4_joint"]),  # end-effector
            "entity_b": SceneEntityCfg("pick_cube"),
        }

        self.rewards.lift_cube = {
            "weight": 3.0,
            "height_threshold": 0.15,
            "entity": SceneEntityCfg("pick_cube"),
        }

        # 관측 설정
        self.observations.policy.obs_terms = [
            "robot_joint_pos", "robot_joint_vel",
            "cube_position", "cube_relative_pose"
        ]

        # commands 안씀
        self.commands.base_velocity = None

@configclass
class G1GraspEnvCfg_PLAY(G1GraspEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 10
        self.observations.policy.enable_corruption = False
        self.events.base_external_force_torque = None