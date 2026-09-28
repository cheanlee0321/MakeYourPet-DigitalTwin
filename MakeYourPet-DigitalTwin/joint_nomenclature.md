# 六足/足式機器人 關節對應與命名規範 (Joint Nomenclature)

本文件整理並記錄足式機器人（如六足機器人 Hexapod / 仿生節肢動物）各關節在不同領域的命名方式、連接部位與運動自由度（DOF）定義。

---

## 關節對應總覽表

| 連接部位 | 關節通俗名稱 | 機器人工程學名稱 | 生物解剖學名稱 | 主要運動維度 |
| :--- | :--- | :--- | :--- | :--- |
| **Trunk（機身） ↔ Coxa（基節）** | Base / Yaw | Hip Yaw Joint | Thoracocoxal joint | 水平左右擺動（轉向 / 前後邁步） |
| **Coxa（基節） ↔ Femur（大腿）** | Hip（髖關節） | Hip Pitch Joint | Coxofemoral joint | 垂直上下起伏（抬腿 / 越野跨障） |
| **Femur（大腿） ↔ Tibia（小腿）** | Knee（膝關節） | Knee Pitch Joint | Femorotibial joint | 垂直屈伸折合（伸展抓地 / 蹬地推進） |

---

## 詳細關節特性與運動說明

### 1. 機身 — 基節關節 (Trunk ↔ Coxa)
* **通俗名稱**：Base / Yaw
* **機器人工程學**：Hip Yaw Joint（偏航關節）
* **生物解剖學**：Thoracocoxal joint（胸-基節關節，節肢動物常見稱呼）
* **旋轉軸向**：通常為垂直軸（Z 軸旋轉）
* **主要功能**：負責腿部的水平平面擺動，控制機器人的航向角（Yaw）、原地旋轉以及前進後退時的水平推動角度。

### 2. 基節 — 大腿關節 (Coxa ↔ Femur)
* **通俗名稱**：Hip（髖關節）
* **機器人工程學**：Hip Pitch Joint（俯仰髖關節）
* **生物解剖學**：Coxofemoral joint（基-股節關節）
* **旋轉軸向**：通常為水平切線軸（Pitch 軸旋轉）
* **主要功能**：控制大腿的升降，負責在擺動相（Swing phase）抬腿跨越障礙物，以及在支撐相（Stance phase）支撐機身負重與調節機身高度。

### 3. 大腿 — 小腿關節 (Femur ↔ Tibia)
* **通俗名稱**：Knee（膝關節）
* **機器人工程學**：Knee Pitch Joint（俯仰膝關節）
* **生物解剖學**：Femorotibial joint（股-脛節關節）
* **旋轉軸向**：與 Hip Pitch 軸通常平行（Pitch 軸旋轉）
* **主要功能**：控制小腿的屈伸，負責足端（Foot/Tarsus）的工作半徑調節、抓地接觸點控制以及提供蹬地前進的推進力。
