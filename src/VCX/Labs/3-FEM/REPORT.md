# Lab 3: 基于有限元方法的软体仿真 — 实验报告

## 概述

本实验基于线性有限元方法，实现了**三维软体仿真**和**布料模拟**。支持三种超弹性本构模型（StVK、Neo-Hookean、Corotated）和两种时间积分方案（显式 Symplectic Euler、隐式 Backward Euler + Newton-CG），所有选项均可在 UI 中实时切换。

---

## 1. 软体模拟

### 系统架构

```
CaseFEMSoftBody  ──owns──▶  TetMesh       (几何与拓扑)
       │                    FEMIntegrator  (弹性力 + 切线刚度)
       │
       ├── AdvanceExplicit()   — Symplectic Euler 子步积分
       ├── AdvanceImplicit()   — Backward Euler + Newton-CG
       └── OnRender()          — 光照渲染 / 平面渲染 + 线框
```

### 核心数据结构

- **`TetMesh`**：存储参考/变形位置、速度、集中质量、固定标记、四面体顶点索引（每 tet 4 个）、预处理 $\mathbf{D}_m^{-1}$ 和参考体积 $V_0 = |\det\mathbf{D}_m|/6$。
- **`FEMIntegrator`**：计算弹性力和单元切线刚度矩阵。

### 网格生成 (`TetMesh::BuildBeam`)

长方体梁离散为 $(n_x+1)\times(n_y+1)\times(n_z+1)$ 个顶点和 $n_x n_y n_z \times 6$ 个四面体，采用 **Kuhn 三角剖分**——每个立方体单元沿对角线 $v_{000}\rightarrow v_{111}$ 剖分为 6 个四面体，全部具有正参考体积。

### FEM 计算流程

对每个四面体（顶点 $\mathbf{x}_0,\mathbf{x}_1,\mathbf{x}_2,\mathbf{x}_3$）：

| 物理量 | 公式 | 维度 |
|---|---|---|
| 变形梯度 | $\mathbf{F} = [\mathbf{x}_{10},\mathbf{x}_{20},\mathbf{x}_{30}] \cdot \mathbf{D}_m^{-1}$ | $3\times3$ |
| Green 应变 (StVK) | $\mathbf{G} = \frac{1}{2}(\mathbf{F}^T\mathbf{F} - \mathbf{I})$ | $3\times3$ |
| 第二 PK 应力 (StVK) | $\mathbf{S} = 2\mu\mathbf{G} + \lambda\,\text{tr}(\mathbf{G})\mathbf{I}$ | $3\times3$ |
| 第一 PK 应力 | $\mathbf{P} = \mathbf{F}\mathbf{S}$ | $3\times3$ |
| 节点力 | $[\mathbf{f}_1,\mathbf{f}_2,\mathbf{f}_3] = -V_0 \mathbf{P} \mathbf{D}_m^{-T},\; \mathbf{f}_0 = -\sum\mathbf{f}_i$ | 4×3 |

### 用户交互

| 输入 | 效果 |
|---|---|
| 空格键按住 | 对所有自由顶点施加向上的抬升力 |
| Alt + 鼠标拖拽 | 通过 `ForceManager` 给最近顶点施加冲量 |
| UI 滑条 | $\lambda$、$\mu$、质量、阻尼、重力、子步数等 |

### 渲染

- **光照表面**：Blinn-Phong 方向光（`lit_mesh.vert`/`lit_mesh.frag`），支持光强/环境光/高光度/Flat-Smooth 切换
- **线框**：Flat shader 叠加（去重边）
- 可通过复选框完全关闭光照，回到最初的纯色渲染

---

## 2. B1: 多种超弹性模型对比 ★☆☆

### 实现

三种本构模型在 `FEMIntegrator::ComputeElementForces` 中实现，通过 Physics 面板的 `ImGui::Combo` 下拉菜单实时切换：

| 模型 | 第一 PK 应力 $\mathbf{P}$ | 特点 |
|---|---|---|
| **StVK** | $\mathbf{P} = \mathbf{F}[2\mu\mathbf{G} + \lambda\,\text{tr}(\mathbf{G})\mathbf{I}]$ | 简单经典，但大旋转下会产生虚假硬化 |
| **Neo-Hookean** | $\mathbf{P} = \mu(\mathbf{F} - \mathbf{F}^{-T}) + \lambda\ln J\,\mathbf{F}^{-T}$ | 多凸（polyconvex），大变形下更稳定 |
| **Corotated** | $\mathbf{P} = \mathbf{R}[2\mu(\mathbf{S}-\mathbf{I}) + \lambda\,\text{tr}(\mathbf{S}-\mathbf{I})\mathbf{I}]$ | 极分解去旋转，旋转不变 |

Corotated 模型通过 SVD 极分解 $\mathbf{F}=\mathbf{R}\mathbf{S}$ 提取旋转部分 $\mathbf{R}$，仅在拉伸 $\mathbf{S}$ 上施加线性弹性后再旋转回世界系，从而消除了 StVK 在旋转下的虚假硬化效应。

对布料仿真也实现了相同的三种模型，区别在于变形梯度为 $3\times2$（详见 §3）。

### UI

Physics 面板中 `Model` 下拉框选择 `StVK` / `Neo-Hookean` / `Corotated`，切换即时生效，无需重建网格。

---

## 3. B2: 布料模拟 ★★☆

### 架构

```
CaseCloth  ──owns──▶  ClothMesh           (2D 参数坐标 + 3D 变形位置)
       │             ClothFEMIntegrator   (弹性力, 3×2 F)
       │
       └── Advance()   — 显式 Symplectic Euler + Rayleigh 阻尼
```

### 与四面体 FEM 的关键区别

| 方面 | 软体（四面体） | 布料（三角面片） |
|---|---|---|
| 单元类型 | 四面体（4 节点） | 三角形（3 节点） |
| $\mathbf{F}$ | $3\times3$ | $3\times2$ |
| $\mathbf{D}_m$ | $3\times3$（参考边向量） | $2\times2$（UV 参数空间边向量） |
| 应变 $\mathbf{G}$ | $3\times3$ | $2\times2$ |
| 面积/体积因子 | $V_0 = |\det\mathbf{D}_m|/6$ | $A_0 = |\det\mathbf{D}_m|/2$ |
| 节点力 | $[\mathbf{f}_1,\mathbf{f}_2,\mathbf{f}_3] = -V_0\mathbf{P}\mathbf{D}_m^{-T}$ | $[\mathbf{f}_1,\mathbf{f}_2] = -A_0\mathbf{P}\mathbf{D}_m^{-T}$ |

### 材料参数

布料使用平面应力假设，由杨氏模量 $E$ 和泊松比 $\nu$ 导出平面应力 Lamé 参数：

$$\mu = \frac{E}{2(1+\nu)}, \qquad \lambda = \frac{E\nu}{1-\nu^2}$$

### 阻尼

布料采用两种阻尼：
1. **质量比例阻尼**：$v_i \leftarrow v_i \cdot e^{-k_d \Delta t}$（指数衰减，无条件稳定）
2. **刚度比例阻尼**（Rayleigh）：$\mathbf{f}_{\text{damp}} = -A_0 \beta \dot{\mathbf{P}} \mathbf{D}_m^{-T}$，其中 $\dot{\mathbf{P}}$ 由速度变形梯度率 $\dot{\mathbf{F}} = \mathbf{D}_v \mathbf{D}_m^{-1}$ 计算，通过 StVK 线性化近似

### 交互方式

- **空格键按住**：吹风效果（+Z 方向为主，微向上分量）
- **Alt + 鼠标拖拽**：`ForceManager` 冲量
- **固定角点**：可通过 UI 选择固定布料上方两角，形成悬挂效果

### 稳定性措施

- 退化检测：跳过 $\det(\mathbf{F}^T\mathbf{F}) < 10^{-8}$ 的三角面片
- 速度钳制：上限 100 m/s
- NaN 检测：自动将异常顶点重置为参考构型

---

## 4. B4: 隐式时间积分 ★★★

### 算法：Backward Euler + Newton-CG

离散运动方程：

$$\mathbf{v}_{n+1} = \mathbf{v}_n + \Delta t\,\mathbf{M}^{-1}\mathbf{f}(\mathbf{x}_{n+1})$$
$$\mathbf{x}_{n+1} = \mathbf{x}_n + \Delta t\,\mathbf{v}_{n+1}$$

合并为关于 $\mathbf{x}_{n+1}$ 的单一方程：

$$\mathbf{M}(\mathbf{x}_{n+1} - \mathbf{x}_n - \Delta t\mathbf{v}_n) - \Delta t^2\mathbf{f}(\mathbf{x}_{n+1}) = \mathbf{0}$$

在当前迭代点 $\mathbf{x}^{(k)}$ 处 Newton 线性化：

$$(\mathbf{M} + \Delta t^2 \mathbf{K}^{(k)}) \Delta\mathbf{x} = \Delta t^2 \mathbf{f}(\mathbf{x}^{(k)}) - \mathbf{M}(\mathbf{x}^{(k)} - \mathbf{x}_n - \Delta t\mathbf{v}_n)$$

其中 $\mathbf{K}^{(k)} = -\partial\mathbf{f}/\partial\mathbf{x}$ 为 $\mathbf{x}^{(k)}$ 处的**切线刚度矩阵**。

### 单元切线刚度矩阵

`FEMIntegrator::ComputeElementTangentStiffness` 通过解析微分计算 $12\times12$ 的 $\mathbf{K}_e$：

对顶点 $j$ 方向 $b$ 的单位扰动 $\delta\mathbf{x}_j = \mathbf{e}_b$，通过链式法则：

$$\delta\mathbf{F} = \mathbf{e}_b \cdot \mathbf{g}_j^T, \quad \mathbf{g}_j = \mathbf{D}_m^{-T} \cdot \mathbf{c}_j$$

其中 $\mathbf{c}_j$ 为顶点 $j$ 在四面体中的选择向量：
$$\mathbf{c}_0=(-1,-1,-1)^T,\;\mathbf{c}_1=(1,0,0)^T,\;\mathbf{c}_2=(0,1,0)^T,\;\mathbf{c}_3=(0,0,1)^T$$

$$\delta\mathbf{G} = \tfrac{1}{2}(\mathbf{g}_j \cdot \mathbf{F}_{row(b)} + \mathbf{F}_{row(b)}^T \cdot \mathbf{g}_j^T)$$

$$\delta\mathbf{S} = 2\mu\,\delta\mathbf{G} + \lambda\,\text{tr}(\delta\mathbf{G})\mathbf{I}$$

$$\delta\mathbf{P} = \delta\mathbf{F}\cdot\mathbf{S} + \mathbf{F}\cdot\delta\mathbf{S}$$

$$\mathbf{K}_e(3i+a, 3j+b) = -\delta\mathbf{f}_i^a = V_0 \cdot [\delta\mathbf{P}\cdot\mathbf{D}_m^{-T}] \cdot \mathbf{w}_i$$

其中 $\mathbf{w}_i$ 为力选择向量：$\mathbf{w}_0=(-1,-1,-1)^T,\;\mathbf{w}_1=(1,0,0)^T,\;\mathbf{w}_2=(0,1,0)^T,\;\mathbf{w}_3=(0,0,1)^T$。

### 全局装配与 CG 求解

1. **质量矩阵**：集中质量对角阵 $\mathbf{M}_i = m_i \mathbf{I}_{3\times3}$
2. **全局系统**：$\mathbf{A} = \mathbf{M} + \Delta t^2 \mathbf{K}$，使用 `Eigen::SparseMatrix<float>` 逐单元装配
3. **边界条件**：固定顶点从系统中移除（对应行列置零，对角设为 1）
4. **CG 求解**：`Eigen::ConjugateGradient` + `DiagonalPreconditioner`，容差和最大迭代次数可配置

### 安全措施

| 措施 | 说明 |
|---|---|
| Δx 钳制 | 每 Newton 迭代中顶点位移上限 0.5 m，防止 CG 未完全收敛时顶点飞出 |
| CG 失败回退 | 若 CG 数值失败，自动回退到显式 Symplectic Euler 完成本帧积分 |
| 收敛指示 | UI 中显示绿色 "CG: converged" 或红色 "CG: NOT converged" |
| NaN 保护 | 检测位置/速度的 NaN/Inf，自动重置为参考构型 |

### UI 控件（Simulation 面板）

| 控件 | 默认值 | 说明 |
|---|---|---|
| Implicit | 关闭 | 切换隐式/显式积分 |
| Newton Iters | 5 | 每帧最大 Newton 迭代次数 |
| Max CG Iters | 200 | CG 求解器迭代上限 |
| CG Tolerance | $10^{-4}$ | CG 收敛容差 |

---

## 5. 开发过程中的困难与解决

### 5.1 力的公式错误

**问题**：初始实现中 `FEMIntegrator.cpp` 的节点力计算使用了 `DmInv.transpose().inverse()`，即 $\mathbf{D}_m^{-T}$ 再求逆，实际等于 $\mathbf{D}_m^T$。正确公式要求 $\mathbf{D}_m^{-T}$（仅转置，不再求逆）。由于 $\mathbf{D}_m^T$ 量级约 0.25，而 $\mathbf{D}_m^{-T}$ 量级约 4，旧代码的弹性力**小了约 16 倍**。修正后力变大了两个数量级，导致原本勉强稳定的参数完全失效。

**解决**：移除多余的 `.inverse()`，同时将默认 $\lambda$ 从 1000 降到 300、$\mu$ 从 500 降到 50，以补偿力的大幅增加。



### 5.2 布料仿真的 NaN 处理

**问题**：布料仿真在初期频繁出现黑屏或剧烈抖动。经排查有三个原因：

1. **初始杨氏模量过高**：默认 $E=50000$ Pa 导出 $\lambda\approx16484, \mu\approx19231$，是软体的 300 倍。在 0.1 kg 的极轻布料上，1% 应变产生的加速度高达 $4\times10^4$ m/s²。将 $E$ 降至 50 Pa 。


2. **缺少 NaN 兜底防护**：即使修复了上述问题，极端情况下（如鼠标突然大力拖拽）仍可能出现 NaN。在 `Advance` 循环中加入 `allFinite()` 检查和自动重置逻辑后，即使偶尔出现异常顶点也能自动恢复，不再黑屏。

### 5.3 隐式积分的 CG 收敛问题

**问题**：CG 求解器在材料较硬（$\lambda,\mu$ 大）时条件数恶化，200 次迭代内无法收敛到指定容差，导致 Δx 不准确，个别帧出现"炸开"现象（位置突然大幅偏离，下一帧又恢复）。

**解决**：
- 加入对角预条件子（`DiagonalPreconditioner`），利用 $(\mathbf{M}+\Delta t^2\mathbf{K})$ 的对角占优特性加速收敛
- 每 Newton 迭代钳制顶点位移上限为 0.5 m，防止 CG 近似解产生灾难性位置更新
- CG 完全失败时回退到显式积分完成本帧


---

## 6. 代码文件结构

```
src/VCX/Labs/3-FEM/
├── App.h / App.cpp                    # IApp 入口，Case 注册
├── main.cpp                           # Engine::RunApp
├── TetMesh.h / TetMesh.cpp            # 四面体网格：BuildBeam, ExtractSurfaceFaces, FixTopFace
├── FEMIntegrator.h / FEMIntegrator.cpp # 弹性力（3 种模型）+ 切线刚度矩阵
├── CaseFEMSoftBody.h / CaseFEMSoftBody.cpp # 软体 Case：显式+隐式、渲染、UI
├── ClothMesh.h / ClothMesh.cpp        # 布料网格：BuildGrid, FixCorner
├── ClothFEMIntegrator.h / ClothFEMIntegrator.cpp # 布料力（3 种模型, 3×2 F）+ Rayleigh 阻尼
├── CaseCloth.h / CaseCloth.cpp        # 布料 Case：仿真、渲染、UI
└── REPORT.md                          # 本报告

assets/shaders/
├── flat.vert / flat.frag              # 平面纯色着色器
└── lit_mesh.vert / lit_mesh.frag      # Blinn-Phong 光照着色器（Flat/Smooth 切换）
```

---

## 7. Demo 展示

### 软体模拟（显式积分，StVK 模型）

> [插入截图：软体梁平放地面，按住空格键抬升的效果]

### 三种本构模型对比（StVK / Neo-Hookean / Corotated）

> [插入截图：三种模型在同条件下的变形差异对比]

### 布料模拟（悬挂 + 吹风）

> [插入截图：布料从两角悬挂，空格键吹风飘动效果]

### 隐式积分 UI 面板（含 CG 收敛状态）

> [插入截图：隐式模式 UI 面板，显示 Newton Iters、CG Iters、CG Tolerance 和收敛状态]

### 光照效果对比（Flat Shading / Smooth Shading / 无光照）

> [插入截图：光照开关、Flat/Smooth 切换效果对比]

---
