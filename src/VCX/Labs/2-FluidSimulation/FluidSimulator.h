#pragma once

#include <vector>
#include <Eigen/Dense>
#include "BoundaryConditions.h"

namespace VCX::Labs::Fluid {

// ── 数据结构（直接放在这个头文件里，不单独拆文件）──
struct Particle {
    Eigen::Vector2f pos;
    Eigen::Vector2f vel;
};

struct MACGrid {
    int   nx, ny;
    float h;
    std::vector<float> u, v;        // 速度场（交错网格）
    std::vector<float> uOld, vOld;  // FLIP 用的旧速度快照
    std::vector<float> p;           // 压力场
    std::vector<int>   cellType;    // 0=air 1=fluid 2=solid

    void resize(int _nx, int _ny, float _h);
    int uIdx(int i,int j) const { return j*(nx+1)+i; }
    int vIdx(int i,int j) const { return j*nx+i; }
    int cIdx(int i,int j) const { return j*nx+i; }
};

// ── 主仿真器 ──
class FluidSimulator {
public:
    // 公开参数（CaseFlip 通过 ImGui 直接修改这些）
    float     flipRatio  = 0.95f;
    float     gravity    = -9.8f;
    int       solverIter = 50;

    std::vector<Particle> particles;
    MACGrid               grid;

    FluidSimulator(int nx, int ny, float h);

    // 你规划的主循环，CaseFlip::Advance 只调这一个函数
    void runSubStepLoop(int numSubSteps, float sdt,
                        const BoundaryConditions& bc);

private:
    void integrateParticles(float sdt);
    void handleParticleCollisions(const BoundaryConditions& bc);
    void transferVelocities(bool toGrid, float flipRatio = 0.95f);
    void solveIncompressibility();
};

} // namespace VCX::Labs::Fluid