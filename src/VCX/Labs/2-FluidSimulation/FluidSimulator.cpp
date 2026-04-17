#include "FluidSimulator.h"

namespace VCX::Labs::Fluid {

void MACGrid::resize(int _nx, int _ny, int _nz, float _h) {
    nx = _nx; ny = _ny; nz = _nz; h = _h;
    u.assign((nx+1)*ny*nz, 0.f);
    v.assign(nx*(ny+1)*nz, 0.f);
    w.assign(nx*ny*(nz+1), 0.f);
    uOld = u; vOld = v; wOld = w;
    p.assign(nx*ny*nz, 0.f);
    cellType.assign(nx*ny*nz, 0);
}

FluidSimulator::FluidSimulator(int nx, int ny, int nz, float h) {
    grid.resize(nx, ny, nz, h);
}

// ── 主循环（你规划的那段代码直接放这里）──
void FluidSimulator::StimulateTimestep(float const dt) {
    for (int step = 0; step < numSubSteps; ++step) {
        integrateParticles(dt);
        handleParticleCollisions(bc);
        transferVelocities(true);            // P2G
        solveIncompressibility();
        transferVelocities(false, flipRatio); // G2P
    }
}

void FluidSimulator::integrateParticles(float const dt) {
    for (auto& p : particles) {
        p.vel.y() += gravity * dt;   // 重力
        p.pos     += dt * p.vel;     // 欧拉积分
    }
}

void FluidSimulator::handleParticleCollisions(const BoundaryConditions& bc) {
    for (auto& p : particles) {
        // 夹到边界内
        p.pos = p.pos.cwiseMax(bc.minBound).cwiseMin(bc.maxBound);
        // Neumann: 碰到边界将法向速度归零
        if (bc.type == BCType::Neumann) {
            if (p.pos.x() <= bc.minBound.x() || p.pos.x() >= bc.maxBound.x())
                p.vel.x() = 0.f;
            if (p.pos.y() <= bc.minBound.y() || p.pos.y() >= bc.maxBound.y())
                p.vel.y() = 0.f;
        }
    }
}

void FluidSimulator::transferVelocities(bool toGrid, float flipRatio) {
    if (toGrid) {
        // P2G：粒子速度加权插值到网格
        // 同时保存旧速度快照供 FLIP 使用
        grid.uOld = grid.u;
        grid.vOld = grid.v;
        // ... 双线性插值实现 ...
    } else {
        // G2P：根据 flipRatio 混合 PIC 和 FLIP
        for (auto& p : particles) {
            // v_pic  = 从当前网格插值
            // dv     = 从(新-旧)网格插值
            // p.vel  = (1-flipRatio)*v_pic + flipRatio*(p.vel + dv)
        }
    }
}

void FluidSimulator::solveIncompressibility() {
    // Gauss-Seidel 迭代消除散度
    for (int iter = 0; iter < solverIter; ++iter) {
        for (int j = 1; j < grid.ny-1; ++j)
            for (int i = 1; i < grid.nx-1; ++i) {
                if (grid.cellType[grid.cIdx(i,j)] != 1) continue;
                // 计算散度，分配压力修正，更新相邻速度
            }
    }
}

} // namespace