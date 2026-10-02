import os
import sys

# 確保 Windows cp950 環境下能夠正確輸出 UTF-8 與表情符號 (避免 torch.onnx 印出 checkmark 報錯)
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8")

import argparse
import numpy as np
import torch
import onnx
import onnxruntime as ort
from stable_baselines3 import PPO

class HexapodActor(torch.nn.Module):
    """提取 PPO 策略網路之 Actor 部分，供邊緣裝置與 Sim-to-Real 推論"""
    def __init__(self, policy):
        super().__init__()
        self.mlp_extractor = policy.mlp_extractor.policy_net
        self.action_net = policy.action_net

    def forward(self, obs):
        latent = self.mlp_extractor(obs)
        action = self.action_net(latent)
        # 截斷在 [-1.0, 1.0]
        return torch.clamp(action, -1.0, 1.0)

def parse_args():
    parser = argparse.ArgumentParser(description="將訓練完成之六足步態策略匯出為 ONNX 格式")
    parser.add_argument("--model", type=str, default=None,
                        help="欲匯出的模型路徑 (預設依序嘗試 models/best_model/best_model.zip 或 models/hexapod_final_policy.zip)")
    parser.add_argument("--output", type=str, default="models/hexapod_policy.onnx",
                        help="輸出的 ONNX 模型路徑")
    return parser.parse_args()

def main():
    args = parse_args()

    base_models_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "models"))
    candidate_paths = [
        args.model,
        os.path.join(base_models_dir, "best_model", "best_model.zip"),
        os.path.join(base_models_dir, "hexapod_final_policy.zip"),
        os.path.join("models", "best_model", "best_model.zip"),
        os.path.join("models", "hexapod_final_policy.zip")
    ]
    output_path = args.output
    if output_path == "models/hexapod_policy.onnx" and os.path.exists(base_models_dir):
        output_path = os.path.join(base_models_dir, "hexapod_policy.onnx")

    model_path = None
    for p in candidate_paths:
        if p and os.path.exists(p):
            model_path = p
            break

    if not model_path:
        print(f"[錯誤] 找不到可匯出的模型！請先執行 train.py 完成訓練。")
        return

    print("=" * 65)
    print(f"正在載入模型: {model_path}")
    model = PPO.load(model_path, device="cpu")
    actor = HexapodActor(model.policy)
    actor.eval()

    dummy_input = torch.zeros((1, 67), dtype=torch.float32)

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    print(f"正在匯出 ONNX 至: {output_path} ...")

    torch.onnx.export(
        actor,
        dummy_input,
        output_path,
        input_names=["observation"],
        output_names=["action"],
        dynamic_axes={"observation": {0: "batch_size"}, "action": {0: "batch_size"}},
        opset_version=17
    )

    # 驗證 ONNX 模型完整性
    onnx_model = onnx.load(output_path)
    onnx.checker.check_model(onnx_model)
    file_size_kb = os.path.getsize(output_path) / 1024.0
    print(f"ONNX 模型驗證通過！檔案大小: {file_size_kb:.1f} KB")

    # 執行 ONNX Runtime 實測
    session = ort.InferenceSession(output_path)
    ort_inputs = {session.get_inputs()[0].name: np.random.randn(1, 67).astype(np.float32)}
    ort_outs = session.run(None, ort_inputs)
    print(f"ONNX Runtime 推論測試成功，輸出動作形狀: {ort_outs[0].shape}")
    print("=" * 65)

if __name__ == "__main__":
    main()
