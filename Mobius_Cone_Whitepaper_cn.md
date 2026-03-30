
---

# 莫比烏斯錐：一種基於仿生流形與動態基準導航的無奇點運動控制架構
**Mobius-Cone: A Biomimetic Manifold Framework for Singularity-Free Motion Control**

**作者：** [creatxrgithub](https://github.com/creatxrgithub)
**類別：** 計算機圖形學 / 生物動力學 / 機器人控制

---

## 一、 摘要 (Abstract)

本文提出「莫比烏斯錐（Mobius-Cone）」運動控制架構，旨在根本性解決 3D 骨骼系統在極限姿態下的萬向節死鎖（Gimbal Lock）與幾何奇點問題。本架構主張肢體動作是由骨骼相對於特定中軸的**夾角變換（Swing）**與**合成旋轉（Rotation）**構成。透過在「面向系」導航下靈活配置「身外」與「身上」雙重基準中軸，並引入「行為分配與棧式執行」的分層驅動管線，實現了具備拓撲連續性且符合生物直覺的運動映射。

---

## 二、 命名由來與幾何直覺 (Nomenclature and Intuition)

本架構的核心靈感源於**「莫比烏斯環（Möbius strip）」**的拓撲特性。

在生物動力學觀察中，人體或有骨骼動物的動作，皆由骨骼改變其與某中軸的夾角及旋轉角度合成。
* **翻轉特性：** 在外觀軌跡上，每根骨骼的活動軌跡並非單純的圓錐，而是一個會隨旋轉連續翻轉「內外」側的流形空間。
* **定義：** 這種受解剖約束、且在旋轉中具備翻轉特性的運動軌跡，形成了一個受限的**「類莫比烏斯環錐體」**，故稱之為「莫比烏斯錐」。

---

## 三、 面向系導航與雙重基準設計 (Navigation and Reference Design)

### 3.1 面向系動態座標 (Heading-based Coordinates)
為了確保在 360 度任意空間姿態下皆能穩定導航，系統透過採樣主體特徵（如大椎、肩峰、下巴骨）定義「瞬時面向基底」。指令不再相對於外部世界，而是相對於主體當前的「前後左右」，從源頭規避了因世界座標軸重合導致的死鎖現象。

![shoulder.png](shoulder.png)

### 3.2 靈活的中軸配置 (Adaptive Axis Configuration)
* **肩錐模式（身外基準 / External）：** 中軸定於相對於「面向系」的固定偏置上（如左肩向正前方左側轉 **35°**）。這為高自由度關節提供了穩定的「幾何底座」，不隨骨骼自轉而翻轉。
* **肘錐模式（身上基準 / Local）：** 中軸定於「身上」的骨骼段矢量（肘 $\to$ 腕 與 肘 $\to$ 肩）。透過局部基準，精確實現小臂自轉軸與彎曲軸的非共軸合成。

![elbow.png](elbow.png)

---

## 四、 系統架構：行為分配與棧式執行管線 (System Pipeline)

`Mobius-Cone-IK` 透過「決策與執行分離」的策略，確保動作的平滑度與可組合性：

1.  **行為分配層 (Behavior Allocation)：** 行為函數（Behavior Function）**不執行行為**，僅根據意圖（Context）分配參數給骨節點對象，並存入 `node_config` 棧中。
2.  **軌跡協調層 (Trajectory Coordination)：** 核心緩衝區。在參數傳遞過程中，調用協調函數進行時間域平滑、速度序列分配（如：30% 慢速 - 40% 快速）以及物理阻尼計算。
3.  **棧式執行層 (Stacked Execution)：** 統一的 FK 執行函數遍歷配置棧，進行幾何合成並最終驅動骨骼矩陣。

---

## 五、 核心幾何實現 (Core Geometric Implementation)

### 5.1 座標系無關的局部基構造 (Algorithm 001)
這是整套架構的**幾何原點**。透過「最小分量擾動法」動態生成正交輔助軸 $u$，確保在任何朝向下叉積運算皆不退化。

```gdscript
static func get_bone_mobius_cone_quaternion(skel: Skeleton3D, bone_name: String, rotate_angle: float, swing_angle: float, center_axis: Vector3) -> Quaternion:
    var bone_idx = skel.find_bone(bone_name)
    if bone_idx == -1: return Quaternion.IDENTITY

    var r_rad = deg_to_rad(rotate_angle)
    var s_rad = deg_to_rad(swing_angle)
    var n = center_axis.normalized()

    # 座標系無關的局部基構造 (Dynamic Basis Construction)
    var u: Vector3
    var abs_n = n.abs()

    if abs_n.x <= abs_n.y and abs_n.x <= abs_n.z:
        u = n.cross(Vector3(n.x + 1.0, n.y, n.z)).normalized()
    elif abs_n.y <= abs_n.z:
        u = n.cross(Vector3(n.x, n.y + 1.0, n.z)).normalized()
    else:
        u = n.cross(Vector3(n.x, n.y, n.z + 1.0)).normalized()

    var q_swing = Quaternion(u, s_rad)
    var q_rotate = Quaternion(n, r_rad)
    return q_rotate * q_swing
```

### 5.2 骨骼段矢量提取 (Algorithm 003)
負責從靜止位（Rest Pose）中提取具備魯棒性的方向矢量。
```gdscript
static func get_bone_segment_vector(skel: Skeleton3D, p_name: String, c_name: String) -> Vector3:
    var rel_pos = skel.get_bone_rest(skel.find_bone(c_name)).origin
    if rel_pos.length() < 0.0001:
        return skel.get_bone_rest(skel.find_bone(p_name)).basis.y.normalized()
    return rel_pos.normalized()
```

---

## 六、 標準化行為接口與執行規範 (Standardized Interface)

### 6.1 原子行為分配器 (Behavior Allocator)
行為函數僅負責參數的分配與預處理。

```gdscript
# Algorithm 011: 行為函數範例
static func apply_mobius_behavior_xxx(skel: Skeleton3D, context: Dictionary, node_config: Array, coordinator: Callable = Callable()) -> Array:
    # 根據意圖 context 分配參數給 node_config (若是原子行為則僅一個節點)
    var node = {
        "bone_name": context.get("bone_name"),
        "bend": context.get("target_bend"),
        "rotation": context.get("target_rotate"),
        "axis_rotate": context.get("axis"), # 若無則由執行器辨識
        "speed": context.get("speed", 0.1)
    }
    # 調用協調函數進行二次修正
    if coordinator.is_valid():
        node = coordinator.call(node, context)
    node_config.append(node)
    return node_config
```

### 6.2 棧式 FK 執行器 (Stacked FK Executor)
執行器遍歷 `node_config` 棧，完成從參數到物理轉向的最終合成。

```gdscript
# Algorithm 015: 棧執行函數
static func execute_fk_stack(skel: Skeleton3D, node_config: Array):
    for node in node_config:
        var b_idx = node.get("bone_idx", skel.find_bone(node.bone_name))

        # 1. 獲取中軸：優先使用 axis_func，否則由骨骼名辨別
        var center_axis = node.get("axis_rotate", Vector3.UP)

        # 2. 計算軌跡：若無預設 target_quaternion 則現場計算
        var target_q = node.get("target_quaternion",
            get_bone_mobius_cone_quaternion(skel, node.bone_name, node.rotation, node.bend, center_axis)
        )

        # 3. 軌跡協調與速度分配執行
        var current_q = skel.get_bone_pose_rotation(b_idx)
        var final_q = current_q.slerp(target_q, node.get("speed", 0.1))

        skel.set_bone_pose_rotation(b_idx, final_q)
```

---

## 七、 結論 (Conclusion)

莫比烏斯錐架構透過「決策分配」與「統一執行」的解耦，大幅提升了 3D 動畫系統的擴展性與穩定性。該模型不僅從幾何層面消除了萬向節死鎖，更在工程層面建立了一套高效、可疊加的原子級行為規範。

---

## 八、 參考文獻 (References)

1.  **Aristidou, A., et al. (2018).** *Inverse Kinematics: Techniques and Applications.*
2.  **Park, F. C., & Lynch, K. M. (2017).** *Modern Robotics: Mechanics, Planning, and Control.*
3.  **Kuffner, J. (2004).** *Effective sampling and distance metrics for 3D rigid body configurations.*

---

