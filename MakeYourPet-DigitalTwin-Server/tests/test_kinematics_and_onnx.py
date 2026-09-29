#!/usr/bin/env python3
"""
Unit and invariant tests for ONNX neural network policy, kinematics gait calculation, and pulse conversion
"""

import sys
import os
import math
import unittest
import numpy as np
import onnxruntime as ort

TOOLS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "tools"))
if TOOLS_DIR not in sys.path:
    sys.path.insert(0, TOOLS_DIR)

from test_full_loop_simulation import (
    VirtualChicaServer,
    JOINT_TO_PIN_MAP,
    DEFAULT_STAND_PULSES,
    LEG_IS_TRIPOD_A,
    LEG_IS_RIGHT
)

class TestKinematicsAndOnnx(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Locate hexapod_policy.onnx
        cls.model_paths = [
            os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "chica-server-main", "app", "src", "main", "assets", "hexapod_policy.onnx")),
            os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "MakeYourPet-DigitalTwin", "models", "hexapod_policy.onnx")),
        ]
        cls.valid_model_path = None
        for p in cls.model_paths:
            if os.path.exists(p):
                cls.valid_model_path = p
                break
        if cls.valid_model_path is None:
            raise FileNotFoundError(f"hexapod_policy.onnx not found in candidates: {cls.model_paths}")

        cls.session = ort.InferenceSession(cls.valid_model_path)
        cls.input_name = cls.session.get_inputs()[0].name
        cls.output_name = cls.session.get_outputs()[0].name

    def test_onnx_model_io_signatures(self):
        inputs = self.session.get_inputs()
        outputs = self.session.get_outputs()

        self.assertEqual(len(inputs), 1)
        self.assertEqual(len(outputs), 1)

        input_shape = inputs[0].shape
        output_shape = outputs[0].shape

        # Verify batch dimension and feature dimension
        self.assertEqual(input_shape[-1], 67, "Observation vector must have 67 dimensions")
        self.assertEqual(output_shape[-1], 18, "Action vector must have 18 dimensions (18 DOFs)")

    def test_onnx_inference_stability(self):
        # Test 1: Zero observation vector
        zero_obs = np.zeros((1, 67), dtype=np.float32)
        res = self.session.run([self.output_name], {self.input_name: zero_obs})[0]
        self.assertEqual(res.shape, (1, 18))
        self.assertFalse(np.isnan(res).any(), "Inference output must not contain NaN")
        self.assertFalse(np.isinf(res).any(), "Inference output must not contain Inf")

        # Test 2: Extreme observation values
        extreme_obs = np.ones((1, 67), dtype=np.float32) * 10.0
        res_ext = self.session.run([self.output_name], {self.input_name: extreme_obs})[0]
        self.assertFalse(np.isnan(res_ext).any())
        self.assertFalse(np.isinf(res_ext).any())

    def test_compute_q_ref_tripod_symmetry(self):
        # Create an instance of VirtualChicaServer helper
        helper = VirtualChicaServer.__new__(VirtualChicaServer)

        # Test at phase phi = pi / 4 (Tripod A in swing, Tripod B in stance)
        phase = math.pi / 4.0
        q_ref = helper._compute_q_ref(phase, vx=0.35, yaw=0.0)
        self.assertEqual(len(q_ref), 18)

        for leg in range(6):
            base_j = leg * 3
            q_femur = q_ref[base_j + 1]
            q_tibia = q_ref[base_j + 2]

            if LEG_IS_TRIPOD_A[leg]:
                # Tripod A legs should be lifting up (swing: negative femur and tibia angle)
                self.assertLess(q_femur, 0.0, f"Leg {leg+1} (Tripod A) should be lifting in swing phase")
                self.assertLess(q_tibia, 0.0, f"Leg {leg+1} (Tripod A) should be lifting in swing phase")
            else:
                # Tripod B legs should be on ground (stance)
                self.assertGreaterEqual(q_femur, 0.0, f"Leg {leg+1} (Tripod B) should be touching in stance phase")

    def test_compute_q_ref_in_place_yaw_rotation(self):
        helper = VirtualChicaServer.__new__(VirtualChicaServer)
        q_ref = helper._compute_q_ref(phase=0.0, vx=0.0, yaw=0.5)

        # When vx=0 and yaw > 0, left and right coxa swing symmetrically to rotate robot
        # Check Leg 1 (Right) vs Leg 4 (Left)
        coxa_leg1 = q_ref[0]
        coxa_leg4 = q_ref[3 * 3]
        self.assertNotEqual(coxa_leg1, 0.0)
        self.assertNotEqual(coxa_leg4, 0.0)

    def test_angles_to_pulses_neutral_standing(self):
        helper = VirtualChicaServer.__new__(VirtualChicaServer)
        zero_angles = np.zeros(18, dtype=np.float32)
        pulses = helper._angles_to_pulses(zero_angles)

        # At theta=0 rad, all 18 channels must equal DEFAULT_STAND_PULSES exactly
        self.assertEqual(pulses, DEFAULT_STAND_PULSES)
        for pin, p in enumerate(pulses):
            self.assertGreaterEqual(p, 1373, f"Pin {pin} center standing pulse should be >= 1373 us")
            self.assertLessEqual(p, 1627, f"Pin {pin} center standing pulse should be <= 1627 us")

    def test_angles_to_pulses_hardware_clamping_limits(self):
        helper = VirtualChicaServer.__new__(VirtualChicaServer)

        # Extreme positive angles (+1.5 rad ~ +86 deg)
        huge_pos = np.ones(18, dtype=np.float32) * 1.5
        pos_pulses = helper._angles_to_pulses(huge_pos)
        for pin, p in enumerate(pos_pulses):
            self.assertGreaterEqual(p, 700, f"Pin {pin} must be >= 700 us")
            self.assertLessEqual(p, 2300, f"Pin {pin} must be <= 2300 us")

        # Extreme negative angles (-1.5 rad)
        huge_neg = -np.ones(18, dtype=np.float32) * 1.5
        neg_pulses = helper._angles_to_pulses(huge_neg)
        for pin, p in enumerate(neg_pulses):
            self.assertGreaterEqual(p, 700, f"Pin {pin} must be >= 700 us")
            self.assertLessEqual(p, 2300, f"Pin {pin} must be <= 2300 us")

    def test_angles_to_pulses_mirror_symmetry(self):
        helper = VirtualChicaServer.__new__(VirtualChicaServer)

        # Apply a positive pitch angle to Femur on Leg 0 (Left 1) and Leg 3 (Right 1)
        angles = np.zeros(18, dtype=np.float32)
        angles[1] = math.radians(15.0)          # Leg 0 Femur (Left, side_mult=+1.0)
        angles[3 * 3 + 1] = math.radians(15.0)  # Leg 3 Femur (Right, side_mult=-1.0)

        pulses = helper._angles_to_pulses(angles)
        pin_l_femur = JOINT_TO_PIN_MAP[1]          # Pin 16 (L1)
        pin_r_femur = JOINT_TO_PIN_MAP[3 * 3 + 1]  # Pin 13 (R1)

        center_l = DEFAULT_STAND_PULSES[pin_l_femur]
        center_r = DEFAULT_STAND_PULSES[pin_r_femur]

        delta_l = pulses[pin_l_femur] - center_l
        delta_r = pulses[pin_r_femur] - center_r

        # Because of mirrored servo mounting, left delta is positive and right delta is negative
        self.assertGreater(delta_l, 0, "Left femur pulse should increase for positive pitch")
        self.assertLess(delta_r, 0, "Right femur pulse should decrease for positive pitch")
        self.assertAlmostEqual(abs(delta_l), abs(delta_r), delta=2)


if __name__ == "__main__":
    unittest.main()
