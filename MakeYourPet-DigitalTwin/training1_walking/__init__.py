"""
training1_walking: 六足機器人全自由度行走步態訓練與運動學模組
Hexapod robot full-DoF locomotion gait training & kinematics module.
"""
from .hexapod_env import HexapodEnv
from .tripod_kinematics import TripodKinematics

__all__ = ["HexapodEnv", "TripodKinematics"]
