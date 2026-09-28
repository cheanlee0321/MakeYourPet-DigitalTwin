"""
Make Your Pet - 將 Blender 渲染幀序列自動合成為 MP4 高清影片
                Automatically compile Blender rendered frame sequences into high-definition MP4 videos
========================================================================================================
用法 / Usage:
python make_video.py [--fps 50] [--input renders] [--output hexapod_cinematic.mp4]
"""

import os
import glob
import argparse
import cv2


def compile_video(image_dir="renders", output_mp4="hexapod_cinematic.mp4", fps=50):
    images = sorted(glob.glob(os.path.join(image_dir, "*.png")))
    if not images:
        print(f"⚠️ 在 {image_dir} 目錄下未找到任何 PNG 圖片！請先在 Blender 渲染。")
        return

    print(f"🎬 正在合成影片：找到 {len(images)} 幀圖片 (目標幀率: {fps} FPS)...")
    
    first_frame = cv2.imread(images[0])
    h, w, layers = first_frame.shape

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    video = cv2.VideoWriter(output_mp4, fourcc, fps, (w, h))

    for idx, img_path in enumerate(images):
        frame = cv2.imread(img_path)
        video.write(frame)
        if (idx + 1) % 50 == 0 or (idx + 1) == len(images):
            print(f"  合成進度: [{idx + 1}/{len(images)}]")

    video.release()
    print(f"🎉 影片合成完畢！輸出檔案: {os.path.abspath(output_mp4)} ({len(images)/fps:.1f} 秒)")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="幀序列合成 MP4 影片")
    parser.add_argument("--input", type=str, default="renders", help="渲染圖檔目錄")
    parser.add_argument("--output", type=str, default="hexapod_cinematic.mp4", help="輸出影片檔名")
    parser.add_argument("--fps", type=int, default=50, help="動畫幀率 (預設 50)")
    args = parser.parse_args()

    compile_video(args.input, args.output, args.fps)
