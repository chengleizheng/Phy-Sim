# Lab 2: FLIP Fluid Simulation 实验报告

## 一、核心算法思路

### 1.1 算法概述

本项目实现了基于 **FLIP/PIC 混合方法**（Fluid-Implicit-Particle / Particle-in-Cell）的三维不可压缩流体仿真。核心思想是将**拉格朗日视角**（粒子）与**欧拉视角**（MAC 交错网格）结合：粒子携带速度和质量信息，网格负责求解不可压缩性。

### 1.2 数据结构

**MAC 交错网格** (`MACGrid`, `FluidSimulator.h:17-34`)：速度分量存储在面中心而非格心，避免奇偶失耦：

| 分量 | 存储位置 | 维度 |
|------|---------|------|
| `u` (x方向速度) | x-face 中心: `(i·h, (j+0.5)·h, (k+0.5)·h)` | `(nx+1) × ny × nz` |
| `v` (y方向速度) | y-face 中心: `((i+0.5)·h, j·h, (k+0.5)·h)` | `nx × (ny+1) × nz` |
| `w` (z方向速度) | z-face 中心: `((i+0.5)·h, (j+0.5)·h, k·h)` | `nx × ny × (nz+1)` |
| `p` (压力) | 格心: `((i+0.5)·h, (j+0.5)·h, (k+0.5)·h)` | `nx × ny × nz` |

**空间哈希表** (`SpatialHash`, `FluidSimulator.h:37-58`)：加速邻近粒子搜索，将 3D 格坐标哈希为 64 位 key，用于 `pushParticlesApart`。

### 1.3 仿真主循环

`FluidSimulator::StimulateTimestep` (`FluidSimulator.cpp:53-84`) 每子步执行：

```
1. integrateParticles()      → 欧拉积分 (重力 + 位置更新)
2. handleParticleCollisions() → 边界/障碍物碰撞
3. pushParticlesApart()      → 推开重叠粒子 (空间哈希)
4. handleParticleCollisions() → 再次碰撞检测
5. transferVelocities(P2G)   → 粒子速度 → MAC 网格 (三线性加权)
6. updateParticleDensity()   → 标记流体格 + 粒子计数
7. solveIncompressibility()  → 压力投影 (GS 或 CG)
8. transferVelocities(G2P)   → MAC 网格速度 → 粒子 (FLIP/PIC 混合)
```

### 1.4 关键细节

- **FLIP/PIC 混合** (`FluidSimulator.cpp:396-407`)：粒子速度更新为 `(1-α)·v_PIC + α·(v_old + Δv_grid)`，α 由 `flipRatio` 控制（0=纯PIC稳定但耗散大，1=纯FLIP少耗散但易噪声）。
- **三线性插值**：P2G 和 G2P 均使用 8 点加权，权重为距离的线性衰减。
- **速度有效性检查** (`FluidSimulator.cpp:287-301`)：G2P 时只从流体面插值，避免 air-air 界面的虚假速度污染粒子。
- **密度漂移补偿** (`FluidSimulator.cpp:472-476`)：仅对过压缩格（density > restDensity）在散度中加入压缩量，避免欠压缩的正反馈导致流体"炸开"。

---

## 二、交互实现

### 2.1 鼠标拖拽障碍球

使用 `Common::ForceManager` 实现 Alt+鼠标左键拖拽障碍球（`CaseFluid.cpp:106-113`）：

```
ForceManager 计算世界空间位移 → obstaclePos += force
                                 obstacleVel = force / dt
```

粒子碰撞检测 (`FluidSimulator.cpp:112-123`)：当粒子进入障碍球半径内时，沿法向推出并将法向相对速度归零。

### 2.2 UI 控制面板

`CaseFluid::OnSetupPropsUI` (`CaseFluid.cpp:76-99`) 提供三个面板：

| 面板 | 参数 | 说明 |
|------|------|------|
| **Simulation** | dt, flipRatio, pressureIters | 时间步长、FLIP/PIC比例、GS迭代次数 |
| | CG Solver 开关, CG Tolerance | 切换压力求解器类型 |
| | Reset 按钮 | 重新初始化粒子 |
| **Obstacle** | Position (x,y,z) | 障碍球位置（也可鼠标拖拽） |
| **Display** | Color Mode | 速度/密度/压强 三种着色模式 |

### 2.3 相机控制

使用 `OrbitCameraManager` 实现轨道相机：鼠标旋转视角、滚轮缩放。

---

## 三、着色方案

### 3.1 渲染管线

粒子通过 **GPU Instancing** 渲染：每帧在 CPU 构建 offset 数组和 color 数组，使用 `fluid.vert/frag` shader，单个球体几何体 → 批量绘制所有粒子（`CaseFluid.cpp:207-210`）。

### 3.2 三种着色模式

所有模式使用**蓝色渐变**（`浅蓝 (0.55, 0.75, 1.0) → 深蓝 (0.02, 0.08, 0.25)`）：

| 模式 | 映射方式 | 代码位置 |
|------|---------|---------|
| **速度 (Speed)** | `t = clamp(|v|/3.0, 0, 1)` | `CaseFluid.cpp:186-187` |
| **密度 (Density)** | 格心三线性采样 particleDensity，`t = clamp(d/ρ, 0, 2) × 0.5` | `CaseFluid.cpp:188-190` |
| **压强 (Pressure)** | 每帧动态扫描所有流体格的 p_min/p_max，归一化到 [0,1] | `CaseFluid.cpp:165-179, 191-193` |

### 3.3 其他渲染元素

- **域边界** (`CaseFluid.cpp:36-49`)：白色线框 (flat shader)，偏移−0.025 留出视觉边距
- **障碍球** (`CaseFluid.cpp:212-218`)：红色 (0.95, 0.1, 0.1) 球体，单实例渲染
- **光照** (`CaseFluid.cpp:59-66`)：2 个点光源 + 环境光，Blinn-Phong 模型

---

## 四、CG 压力求解器

### 4.1 泊松方程离散化

不可压缩条件 `∇·u = 0` 通过压力投影实现，离散化为 7 点拉普拉斯系统 **A·p = b**：

```
A(对角) = 非固体邻格数  (流体 + 空气，对应 Dirichlet BC)
A(非对角) = -1          (仅流体邻格)
b = -(ρ·h²/Δt) · (∇·u)
```

**边界条件** (`FluidSimulator.cpp:551-559`)：
- **自由液面 (air cell)**：p=0 **Dirichlet** → 计入对角 `cnt++`，但不添加矩阵列（对应齐次边界）
- **固壁 (domain boundary)**：∂p/∂n=0 **Neumann** → 壁面方向不连接，不计入 `cnt`

### 4.2 正确性要点

关键修复 (`FluidSimulator.cpp:551-557`)：
```cpp
auto addNeighbor = [&](int ni, int nj, int nk) {
    cnt++;                    // ★ 所有非固体邻格（含空气）都计入对角
    int nid = fluidToIdx[nc];
    if (nid >= 0)             // 仅流体邻格添加非对角元
        triplets.push_back(T(idx, nid, -1.f));
    // 空气邻格: 计入 cnt 但不加列 → 隐式实现 p=0 边界
};
```

### 4.3 求解与压力梯度修正

使用 `Eigen::ConjugateGradient` 求解 (`FluidSimulator.cpp:585-593`)，收敛容差由 UI 中的 `CG Tolerance` 控制。求解完成后，对每个 interior face 施加速度修正 (`FluidSimulator.cpp:600-627`)：

```
Δu = (sdt / (ρ·h)) · Δp
u_face -= fScale * (p_right - p_left)
```

这种**逐面遍历**的方式避免了格心遍历时每个 face 被修正两次的问题。

### 4.4 CG vs Gauss-Seidel 对比

| 维度 | Gauss-Seidel (SOR) | Conjugate Gradient |
|------|-------------------|-------------------|
| 精度 | 需足量迭代 (50-100) | 设定 tolerance 即可，精度可控 |
| 速度 | 每次迭代 O(N)，但收敛慢 | 收敛快，O(N^(3/2)) 理论 |
| 实现 | 简单就地迭代 | 需构建稀疏矩阵 + Eigen |
| 内存 | O(N) | O(N) 稀疏存储 |
| 自由液面 | 天然支持 (跳过空气格) | 需显式处理 Dirichlet BC |

---

## 五、Demo 展示

> **[在此处粘贴仿真截图]**

### 5.1 速度着色模式

> **[截图 1: 流体下落阶段的速度分布]**

### 5.2 密度着色模式

> **[截图 2: 流体落入底面后的密度分布]**

### 5.3 压强着色模式

> **[截图 3: 压力场的动态范围可视化]**

### 5.4 障碍物交互

> **[截图 4: 鼠标拖拽障碍球与流体互动]**

### 5.5 CG vs GS 对比

> **[截图 5: 相同帧数下两种求解器的效果对比]**

---

## 六、文件结构

```
src/VCX/Labs/2-FluidSimulation/
├── FluidSimulator.h      # MAC 网格 + 粒子结构 + 仿真器声明
├── FluidSimulator.cpp    # 仿真核心实现 (630 行)
├── CaseFluid.h           # UI + 渲染 + 交互 声明
├── CaseFluid.cpp         # UI + 渲染 + 交互 实现 (234 行)
├── App.h / App.cpp       # 应用入口
├── BoundaryConditions.h  # 边界条件工具 (预留)
└── main.cpp              # main 函数
```

## 七、参考资料

- Bridson, R. *Fluid Simulation for Computer Graphics* (2008)
- Zhu & Bridson, *Animating Sand as a Fluid*, SIGGRAPH 2005 (FLIP 方法)
- Stam, J. *Stable Fluids*, SIGGRAPH 1999 (Semi-Lagrangian 对流)
- Eigen 库文档: [ConjugateGradient](https://eigen.tuxfamily.org/dox/classEigen_1_1ConjugateGradient.html)
