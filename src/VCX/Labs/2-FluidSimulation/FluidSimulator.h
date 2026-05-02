#pragma once

#include <vector>
#include <unordered_map>
#include <glm/glm.hpp>
#include <Eigen/Dense>

namespace VCX::Labs::Fluid {

// ── 粒子 ──
struct Particle {
    Eigen::Vector3f pos;
    Eigen::Vector3f vel;
};

// ── MAC 交错网格 ──
struct MACGrid {
    int   nx, ny, nz;
    float h;
    std::vector<float> u, v, w;            // 速度场（交错网格）
    std::vector<float> uOld, vOld, wOld;   // FLIP 旧速度快照
    std::vector<float> p;                  // 压力场
    std::vector<int>   cellType;           // 0=air 1=fluid 2=solid
    std::vector<float> particleDensity;    // 每格粒子数
    float restDensity = 0.f;               // 首帧计算后固定

    void resize(int _nx, int _ny, int _nz, float _h);

    // 交错网格索引函数
    int uIdx(int i, int j, int k) const { return k * (nx + 1) * ny + j * (nx + 1) + i; }
    int vIdx(int i, int j, int k) const { return k * nx * (ny + 1) + j * nx + i; }
    int wIdx(int i, int j, int k) const { return k * nx * ny + j * nx + i; }
    int cIdx(int i, int j, int k) const { return k * nx * ny + j * nx + i; }
};

// ── 空间哈希表（pushParticlesApart 用）──
struct SpatialHash {
    float                                cellSize;
    std::unordered_map<uint64_t, std::vector<int>> table;

    void clear() { table.clear(); }

    // 将 3D 格坐标哈希为 64 位 key
    uint64_t hash(int cx, int cy, int cz) const {
        return (uint64_t(cx) * 73856093) ^ (uint64_t(cy) * 19349663) ^ (uint64_t(cz) * 83492791);
    }

    // 根据粒子位置构建哈希表
    void build(const std::vector<Particle>& particles) {
        table.clear();
        for (int i = 0; i < (int) particles.size(); i++) {
            int cx = int(std::floor(particles[i].pos.x() / cellSize));
            int cy = int(std::floor(particles[i].pos.y() / cellSize));
            int cz = int(std::floor(particles[i].pos.z() / cellSize));
            table[hash(cx, cy, cz)].push_back(i);
        }
    }
};

// ── 主仿真器 ──
class FluidSimulator {
public:
    // ── 公开参数（CaseFluid 通过 ImGui 直接绑定）──
    float flipRatio         = 0.95f;   // 0=PIC, 1=FLIP
    float gravity           = -9.8f;
    int   numPressureIters  = 75;      // Gauss-Seidel 迭代次数
    float overRelaxation    = 1.9f;    // SOR 超松弛因子
    bool  compensateDrift   = true;    // 是否补偿速度漂移
    bool  separateParticles = true;    // 是否启用 pushParticlesApart
    int   numParticleIters  = 2;       // pushParticlesApart 迭代次数
    bool  useCG             = false;   // true=CG泊松求解, false=Gauss-Seidel
    float cgTolerance       = 1e-4f;   // CG 收敛阈值

    // 边界
    Eigen::Vector3f minBound { 0.f, 0.f, 0.f };
    Eigen::Vector3f maxBound { 1.f, 1.f, 1.f };

    std::vector<Particle> particles;
    MACGrid               grid;

    // 障碍物（公开供 CaseFluid 渲染/UI）
    Eigen::Vector3f obstaclePos    { 0.5f, 0.5f, 0.5f };
    float           obstacleRadius = 0.15f;
    Eigen::Vector3f obstacleVel    { 0.f, 0.f, 0.f };

    FluidSimulator(int nx, int ny, int nz, float h);

    // 初始化粒子（在域内生成一块流体）
    void initializeParticles();

    // ── 主循环（供 CaseFluid::Advance 调用）──
    void StimulateTimestep(int numSubSteps, float dt);

private:
    void integrateParticles(float sdt);
    void handleParticleCollisions(const Eigen::Vector3f& obstaclePos,
                                  float                 obstacleRadius,
                                  const Eigen::Vector3f& obstacleVel);
    void pushParticlesApart(int numIters);
    void transferVelocities(bool toGrid, float flipRatio);
    void updateParticleDensity();
    void solveIncompressibility(float sdt);
    void solveIncompressibilityGS(float sdt);
    void solveIncompressibilityCG(float sdt);

    SpatialHash _hash;
};

} // namespace VCX::Labs::Fluid
