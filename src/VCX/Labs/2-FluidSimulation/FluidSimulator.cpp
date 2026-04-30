#include "FluidSimulator.h"
#include <algorithm>
#include <cmath>

namespace VCX::Labs::Fluid {

// ── MACGrid::resize ──
void MACGrid::resize(int _nx, int _ny, int _nz, float _h) {
    nx = _nx; ny = _ny; nz = _nz; h = _h;
    u.assign((nx + 1) * ny * nz, 0.f);
    v.assign(nx * (ny + 1) * nz, 0.f);
    w.assign(nx * ny * (nz + 1), 0.f);
    uOld = u; vOld = v; wOld = w;
    p.assign(nx * ny * nz, 0.f);
    cellType.assign(nx * ny * nz, 0);
    particleDensity.assign(nx * ny * nz, 0.f);
}

// ── 构造函数 ──
FluidSimulator::FluidSimulator(int nx, int ny, int nz, float h) {
    grid.resize(nx, ny, nz, h);
    _hash.cellSize = h * 2.f;  // 哈希格大小为网格间距的 2 倍
}

// ── 粒子初始化：在域上方生成 ~1/6 体积的紧凑流体块 ──
void FluidSimulator::initializeParticles() {
    particles.clear();

    float domainSize = grid.nx * grid.h;
    float margin     = grid.h * 3.f;
    float blockLen   = domainSize * 0.75f;                   // 边长 ≈ 域边长的 75%, 体积 ≈ 1/6
    float spacing    = grid.h * 0.55f;                        // 每格约 8 个粒子

    // x, z 居中, y 贴近顶部
    float xStart = (domainSize - blockLen) * 0.3f;
    float xEnd   = xStart + blockLen;
    float yEnd   = domainSize - margin;
    float yStart = yEnd - blockLen;
    float zStart = (domainSize - blockLen) * 0.3f;
    float zEnd   = zStart + blockLen;

    for (float x = xStart; x < xEnd; x += spacing)
        for (float y = yStart; y < yEnd; y += spacing)
            for (float z = zStart; z < zEnd; z += spacing)
                particles.push_back({ Eigen::Vector3f(x, y, z), Eigen::Vector3f(0, 0, 0) });
}

// ═══════════════════════════════════════════════════════════════════
//  主循环 —— 按照老师建议的结构
// ═══════════════════════════════════════════════════════════════════
void FluidSimulator::StimulateTimestep(int numSubSteps, float dt) {
    float sdt = dt / numSubSteps;

    // 首次运行时计算 restDensity
    if (grid.restDensity == 0.f && !particles.empty()) {
        updateParticleDensity();
        float sum = 0.f;
        int   cnt = 0;
        for (int i = 0; i < grid.nx; i++)
            for (int j = 0; j < grid.ny; j++)
                for (int k = 0; k < grid.nz; k++) {
                    int idx = grid.cIdx(i, j, k);
                    if (grid.cellType[idx] == 1) {
                        sum += grid.particleDensity[idx];
                        cnt++;
                    }
                }
        if (cnt > 0) grid.restDensity = sum / cnt;
    }

    // 默认障碍物：无 (半径=0)
    Eigen::Vector3f obstaclePos(0.5f, 0.3f, 0.5f);
    Eigen::Vector3f obstacleVel(0.f, 0.f, 0.f);

    for (int step = 0; step < numSubSteps; step++) {
        integrateParticles(sdt);
        handleParticleCollisions(obstaclePos, 0.0f, obstacleVel);
        if (separateParticles)
            pushParticlesApart(numParticleIters);
        handleParticleCollisions(obstaclePos, 0.0f, obstacleVel);
        transferVelocities(true, flipRatio);
        updateParticleDensity();
        solveIncompressibility(sdt);
        transferVelocities(false, flipRatio);
    }
}

// ── 1. 重力 + 欧拉积分 ──
void FluidSimulator::integrateParticles(float sdt) {
    for (auto& p : particles) {
        p.vel.y() += gravity * sdt;
        p.pos     += sdt * p.vel;
    }
}

// ── 2. 边界碰撞 + 障碍物碰撞 ──
void FluidSimulator::handleParticleCollisions(const Eigen::Vector3f& obstaclePos,
                                               float                 obstacleRadius,
                                               const Eigen::Vector3f& obstacleVel) {
    for (auto& p : particles) {
        // ── 域边界：夹紧 + 法向速度归零 ──
        for (int d = 0; d < 3; d++) {
            if (p.pos[d] < minBound[d]) {
                p.pos[d] = minBound[d];
                p.vel[d] = 0.f;
            }
            if (p.pos[d] > maxBound[d]) {
                p.pos[d] = maxBound[d];
                p.vel[d] = 0.f;
            }
        }

        // ── 球形障碍物 ──
        if (obstacleRadius > 0.f) {
            Eigen::Vector3f diff = p.pos - obstaclePos;
            float dist           = diff.norm();
            float minDist        = obstacleRadius + grid.h * 0.3f; // 粒子半径 ≈ 0.3h
            if (dist < minDist && dist > 1e-8f) {
                Eigen::Vector3f n = diff / dist;
                p.pos             = obstaclePos + n * minDist;
                // 法向速度相对障碍物归零
                float vn = (p.vel - obstacleVel).dot(n);
                if (vn < 0.f) p.vel -= vn * n;
            }
        }
    }
}

// ── 3. 推开重叠粒子（空间哈希加速）──
void FluidSimulator::pushParticlesApart(int numIters) {
    const float minDist   = grid.h * 0.4f;           // 粒子最小间距
    const float minDistSq = minDist * minDist;

    _hash.cellSize = minDist * 2.f;                  // 哈希格大小 ≥ 搜索半径
    _hash.build(particles);

    for (int iter = 0; iter < numIters; iter++) {
        for (int i = 0; i < (int) particles.size(); i++) {
            int cx = int(std::floor(particles[i].pos.x() / _hash.cellSize));
            int cy = int(std::floor(particles[i].pos.y() / _hash.cellSize));
            int cz = int(std::floor(particles[i].pos.z() / _hash.cellSize));

            // 检查 3×3×3 邻域
            for (int dx = -1; dx <= 1; dx++)
                for (int dy = -1; dy <= 1; dy++)
                    for (int dz = -1; dz <= 1; dz++) {
                        auto it = _hash.table.find(_hash.hash(cx + dx, cy + dy, cz + dz));
                        if (it == _hash.table.end()) continue;

                        for (int j : it->second) {
                            if (i >= j) continue;  // 每对只处理一次

                            Eigen::Vector3f diff = particles[i].pos - particles[j].pos;
                            float distSq         = diff.squaredNorm();
                            if (distSq < minDistSq && distSq > 1e-12f) {
                                float     dist = std::sqrt(distSq);
                                Eigen::Vector3f n = diff / dist;
                                float     overlap = (minDist - dist) * 0.5f;
                                particles[i].pos += n * overlap;
                                particles[j].pos -= n * overlap;
                            }
                        }
                    }
        }
    }
}

// ── 4. 速度传递：P2G / G2P ──
void FluidSimulator::transferVelocities(bool toGrid, float flipRatio) {
    if (toGrid) {
        // ═══ P2G: 粒子速度 → MAC 网格 ═══

        // 清零网格速度 + 权重
        std::vector<float> wu(grid.u.size(), 0.f);
        std::vector<float> wv(grid.v.size(), 0.f);
        std::vector<float> ww(grid.w.size(), 0.f);
        std::fill(grid.u.begin(), grid.u.end(), 0.f);
        std::fill(grid.v.begin(), grid.v.end(), 0.f);
        std::fill(grid.w.begin(), grid.w.end(), 0.f);

        for (auto& p : particles) {
            // 粒子在网格空间中的位置
            float px = p.pos.x() / grid.h;
            float py = p.pos.y() / grid.h;
            float pz = p.pos.z() / grid.h;

            // ── u 面（x 方向交错）──
            {
                int   i0  = int(std::floor(px));
                int   j0  = int(std::floor(py - 0.5f));
                int   k0  = int(std::floor(pz - 0.5f));
                float wx  = px - i0;
                float wy  = (py - 0.5f) - j0;
                float wz  = (pz - 0.5f) - k0;

                i0 = std::clamp(i0, 0, grid.nx - 1); int i1 = std::min(i0+1, grid.nx);
                j0 = std::clamp(j0, 0, grid.ny - 2); int j1 = j0 + 1;
                k0 = std::clamp(k0, 0, grid.nz - 2); int k1 = k0 + 1;

                float w[8] = { (1 - wx) * (1 - wy) * (1 - wz), wx * (1 - wy) * (1 - wz),
                               (1 - wx) * wy * (1 - wz),       wx * wy * (1 - wz),
                               (1 - wx) * (1 - wy) * wz,       wx * (1 - wy) * wz,
                               (1 - wx) * wy * wz,             wx * wy * wz };
                int   is[8] = { i0, i1, i0, i1, i0, i1, i0, i1 };
                int   js[8] = { j0, j0, j1, j1, j0, j0, j1, j1 };
                int   ks[8] = { k0, k0, k0, k0, k1, k1, k1, k1 };

                for (int n = 0; n < 8; n++) {
                    int   idx = grid.uIdx(is[n], js[n], ks[n]);
                    grid.u[idx] += w[n] * p.vel.x();
                    wu[idx]     += w[n];
                }
            }

            // ── v 面（y 方向交错）──
            {
                int   i0  = int(std::floor(px - 0.5f));
                int   j0  = int(std::floor(py));
                int   k0  = int(std::floor(pz - 0.5f));
                float wx  = (px - 0.5f) - i0;
                float wy  = py - j0;
                float wz  = (pz - 0.5f) - k0;

                i0 = std::clamp(i0, 0, grid.nx - 2); int i1 = i0 + 1;
                j0 = std::clamp(j0, 0, grid.ny - 1); int j1 = std::min(j0 + 1, grid.ny);
                k0 = std::clamp(k0, 0, grid.nz - 2); int k1 = k0 + 1;

                float w[8] = { (1 - wx) * (1 - wy) * (1 - wz), wx * (1 - wy) * (1 - wz),
                               (1 - wx) * wy * (1 - wz),       wx * wy * (1 - wz),
                               (1 - wx) * (1 - wy) * wz,       wx * (1 - wy) * wz,
                               (1 - wx) * wy * wz,             wx * wy * wz };
                int   is[8] = { i0, i1, i0, i1, i0, i1, i0, i1 };
                int   js[8] = { j0, j0, j1, j1, j0, j0, j1, j1 };
                int   ks[8] = { k0, k0, k0, k0, k1, k1, k1, k1 };

                for (int n = 0; n < 8; n++) {
                    int   idx = grid.vIdx(is[n], js[n], ks[n]);
                    grid.v[idx] += w[n] * p.vel.y();
                    wv[idx]     += w[n];
                }
            }

            // ── w 面（z 方向交错）──
            {
                int   i0  = int(std::floor(px - 0.5f));
                int   j0  = int(std::floor(py - 0.5f));
                int   k0  = int(std::floor(pz));
                float wx  = (px - 0.5f) - i0;
                float wy  = (py - 0.5f) - j0;
                float wz  = pz - k0;

                i0 = std::clamp(i0, 0, grid.nx - 2); int i1 = i0 + 1;
                j0 = std::clamp(j0, 0, grid.ny - 2); int j1 = j0 + 1;
                k0 = std::clamp(k0, 0, grid.nz - 1); int k1 = std::min(k0 + 1, grid.nz - 1);

                float w[8] = { (1 - wx) * (1 - wy) * (1 - wz), wx * (1 - wy) * (1 - wz),
                               (1 - wx) * wy * (1 - wz),       wx * wy * (1 - wz),
                               (1 - wx) * (1 - wy) * wz,       wx * (1 - wy) * wz,
                               (1 - wx) * wy * wz,             wx * wy * wz };
                int   is[8] = { i0, i1, i0, i1, i0, i1, i0, i1 };
                int   js[8] = { j0, j0, j1, j1, j0, j0, j1, j1 };
                int   ks[8] = { k0, k0, k0, k0, k1, k1, k1, k1 };

                for (int n = 0; n < 8; n++) {
                    int   idx = grid.wIdx(is[n], js[n], ks[n]);
                    grid.w[idx] += w[n] * p.vel.z();
                    ww[idx]     += w[n];
                }
            }
        }

        // 归一化：速度 ÷ 权重
        for (int i = 0; i < (int) grid.u.size(); i++)
            if (wu[i] > 1e-8f) grid.u[i] /= wu[i];
        for (int i = 0; i < (int) grid.v.size(); i++)
            if (wv[i] > 1e-8f) grid.v[i] /= wv[i];
        for (int i = 0; i < (int) grid.w.size(); i++)
            if (ww[i] > 1e-8f) grid.w[i] /= ww[i];

        // ★ 在 P2G 完成之后保存快照（之后 solveIncompressibility 会修改 u/v/w）
        grid.uOld = grid.u;
        grid.vOld = grid.v;
        grid.wOld = grid.w;

    } else {
        // ═══ G2P: MAC 网格速度 → 粒子 (FLIP/PIC 混合) ═══
        for (auto& p : particles) {
            float px = p.pos.x() / grid.h;
            float py = p.pos.y() / grid.h;
            float pz = p.pos.z() / grid.h;

            // ── 三线性插值辅助 lambda ──
            auto sampleU = [&](float x, float y, float z) -> float {
            int   i0 = int(std::floor(x));              // x: 整数方向，无偏移
            int   j0 = int(std::floor(y - 0.5f));       // y: 半整数方向
            int   k0 = int(std::floor(z - 0.5f));       // z: 半整数方向
            float wx = x - i0;
            float wy = (y - 0.5f) - j0;
            float wz = (z - 0.5f) - k0;
            i0 = std::clamp(i0, 0, grid.nx - 1); int i1 = std::min(i0 + 1, grid.nx);     // u: x方向 nx+1 个面
            j0 = std::clamp(j0, 0, grid.ny - 2); int j1 = j0 + 1;
            k0 = std::clamp(k0, 0, grid.nz - 2); int k1 = k0 + 1;
            return (1-wx)*(1-wy)*(1-wz)*grid.u[grid.uIdx(i0,j0,k0)] + wx*(1-wy)*(1-wz)*grid.u[grid.uIdx(i1,j0,k0)]
                + (1-wx)*wy*(1-wz)*grid.u[grid.uIdx(i0,j1,k0)]     + wx*wy*(1-wz)*grid.u[grid.uIdx(i1,j1,k0)]
                + (1-wx)*(1-wy)*wz*grid.u[grid.uIdx(i0,j0,k1)]     + wx*(1-wy)*wz*grid.u[grid.uIdx(i1,j0,k1)]
                + (1-wx)*wy*wz*grid.u[grid.uIdx(i0,j1,k1)]         + wx*wy*wz*grid.u[grid.uIdx(i1,j1,k1)];
            };
            auto sampleV = [&](float x, float y, float z) -> float {
            int   i0 = int(std::floor(x - 0.5f));      // x: 半整数方向
            int   j0 = int(std::floor(y));              // y: 整数方向，无偏移
            int   k0 = int(std::floor(z - 0.5f));       // z: 半整数方向
            float wx = (x - 0.5f) - i0;
            float wy = y - j0;
            float wz = (z - 0.5f) - k0;
            i0 = std::clamp(i0, 0, grid.nx - 2); int i1 = i0 + 1;
            j0 = std::clamp(j0, 0, grid.ny - 1); int j1 = std::min(j0 + 1, grid.ny);     // v: y方向 ny+1 个面
            k0 = std::clamp(k0, 0, grid.nz - 2); int k1 = k0 + 1;
            return (1-wx)*(1-wy)*(1-wz)*grid.v[grid.vIdx(i0,j0,k0)] + wx*(1-wy)*(1-wz)*grid.v[grid.vIdx(i1,j0,k0)]
                + (1-wx)*wy*(1-wz)*grid.v[grid.vIdx(i0,j1,k0)]     + wx*wy*(1-wz)*grid.v[grid.vIdx(i1,j1,k0)]
                + (1-wx)*(1-wy)*wz*grid.v[grid.vIdx(i0,j0,k1)]     + wx*(1-wy)*wz*grid.v[grid.vIdx(i1,j0,k1)]
                + (1-wx)*wy*wz*grid.v[grid.vIdx(i0,j1,k1)]         + wx*wy*wz*grid.v[grid.vIdx(i1,j1,k1)];
            };
            auto sampleW = [&](float x, float y, float z) -> float {
            int   i0 = int(std::floor(x - 0.5f));      // x: 半整数方向
            int   j0 = int(std::floor(y - 0.5f));       // y: 半整数方向
            int   k0 = int(std::floor(z));              // z: 整数方向，无偏移
            float wx = (x - 0.5f) - i0;
            float wy = (y - 0.5f) - j0;
            float wz = z - k0;
            i0 = std::clamp(i0, 0, grid.nx - 2); int i1 = i0 + 1;
            j0 = std::clamp(j0, 0, grid.ny - 2); int j1 = j0 + 1;
            k0 = std::clamp(k0, 0, grid.nz - 1); int k1 = std::min(k0 + 1, grid.nz);     // w: z方向 nz+1 个面
            return (1-wx)*(1-wy)*(1-wz)*grid.w[grid.wIdx(i0,j0,k0)] + wx*(1-wy)*(1-wz)*grid.w[grid.wIdx(i1,j0,k0)]
                + (1-wx)*wy*(1-wz)*grid.w[grid.wIdx(i0,j1,k0)]     + wx*wy*(1-wz)*grid.w[grid.wIdx(i1,j1,k0)]
                + (1-wx)*(1-wy)*wz*grid.w[grid.wIdx(i0,j0,k1)]     + wx*(1-wy)*wz*grid.w[grid.wIdx(i1,j0,k1)]
                + (1-wx)*wy*wz*grid.w[grid.wIdx(i0,j1,k1)]         + wx*wy*wz*grid.w[grid.wIdx(i1,j1,k1)];
            };

            // 同样的 lambda 对旧速度
            auto sampleUOld = [&](float x, float y, float z) -> float {
            int   i0 = int(std::floor(x));
            int   j0 = int(std::floor(y - 0.5f));
            int   k0 = int(std::floor(z - 0.5f));
            float wx = x - i0;
            float wy = (y - 0.5f) - j0;
            float wz = (z - 0.5f) - k0;
            i0 = std::clamp(i0, 0, grid.nx - 1); int i1 = std::min(i0 + 1, grid.nx);
            j0 = std::clamp(j0, 0, grid.ny - 2); int j1 = j0 + 1;
            k0 = std::clamp(k0, 0, grid.nz - 2); int k1 = k0 + 1;
            return (1-wx)*(1-wy)*(1-wz)*grid.uOld[grid.uIdx(i0,j0,k0)] + wx*(1-wy)*(1-wz)*grid.uOld[grid.uIdx(i1,j0,k0)]
                + (1-wx)*wy*(1-wz)*grid.uOld[grid.uIdx(i0,j1,k0)]     + wx*wy*(1-wz)*grid.uOld[grid.uIdx(i1,j1,k0)]
                + (1-wx)*(1-wy)*wz*grid.uOld[grid.uIdx(i0,j0,k1)]     + wx*(1-wy)*wz*grid.uOld[grid.uIdx(i1,j0,k1)]
                + (1-wx)*wy*wz*grid.uOld[grid.uIdx(i0,j1,k1)]         + wx*wy*wz*grid.uOld[grid.uIdx(i1,j1,k1)];
            };
            auto sampleVOld = [&](float x, float y, float z) -> float {
            int   i0 = int(std::floor(x - 0.5f));
            int   j0 = int(std::floor(y));
            int   k0 = int(std::floor(z - 0.5f));
            float wx = (x - 0.5f) - i0;
            float wy = y - j0;
            float wz = (z - 0.5f) - k0;
            i0 = std::clamp(i0, 0, grid.nx - 2); int i1 = i0 + 1;
            j0 = std::clamp(j0, 0, grid.ny - 1); int j1 = std::min(j0 + 1, grid.ny);
            k0 = std::clamp(k0, 0, grid.nz - 2); int k1 = k0 + 1;
            return (1-wx)*(1-wy)*(1-wz)*grid.vOld[grid.vIdx(i0,j0,k0)] + wx*(1-wy)*(1-wz)*grid.vOld[grid.vIdx(i1,j0,k0)]
                + (1-wx)*wy*(1-wz)*grid.vOld[grid.vIdx(i0,j1,k0)]     + wx*wy*(1-wz)*grid.vOld[grid.vIdx(i1,j1,k0)]
                + (1-wx)*(1-wy)*wz*grid.vOld[grid.vIdx(i0,j0,k1)]     + wx*(1-wy)*wz*grid.vOld[grid.vIdx(i1,j0,k1)]
                + (1-wx)*wy*wz*grid.vOld[grid.vIdx(i0,j1,k1)]         + wx*wy*wz*grid.vOld[grid.vIdx(i1,j1,k1)];
            };
            auto sampleWOld = [&](float x, float y, float z) -> float {
            int   i0 = int(std::floor(x - 0.5f));
            int   j0 = int(std::floor(y - 0.5f));
            int   k0 = int(std::floor(z));
            float wx = (x - 0.5f) - i0;
            float wy = (y - 0.5f) - j0;
            float wz = z - k0;
            i0 = std::clamp(i0, 0, grid.nx - 2); int i1 = i0 + 1;
            j0 = std::clamp(j0, 0, grid.ny - 2); int j1 = j0 + 1;
            k0 = std::clamp(k0, 0, grid.nz - 1); int k1 = std::min(k0 + 1, grid.nz);
            return (1-wx)*(1-wy)*(1-wz)*grid.wOld[grid.wIdx(i0,j0,k0)] + wx*(1-wy)*(1-wz)*grid.wOld[grid.wIdx(i1,j0,k0)]
                + (1-wx)*wy*(1-wz)*grid.wOld[grid.wIdx(i0,j1,k0)]     + wx*wy*(1-wz)*grid.wOld[grid.wIdx(i1,j1,k0)]
                + (1-wx)*(1-wy)*wz*grid.wOld[grid.wIdx(i0,j0,k1)]     + wx*(1-wy)*wz*grid.wOld[grid.wIdx(i1,j0,k1)]
                + (1-wx)*wy*wz*grid.wOld[grid.wIdx(i0,j1,k1)]         + wx*wy*wz*grid.wOld[grid.wIdx(i1,j1,k1)];
            };

            Eigen::Vector3f vPIC(sampleU(px, py, pz), sampleV(px, py, pz), sampleW(px, py, pz));

            Eigen::Vector3f vOldAtP(sampleUOld(px, py, pz), sampleVOld(px, py, pz), sampleWOld(px, py, pz));
            Eigen::Vector3f vNewAtP(sampleU(px, py, pz), sampleV(px, py, pz), sampleW(px, py, pz));
            Eigen::Vector3f dv     = vNewAtP - vOldAtP;

            // FLIP/PIC 混合：v = (1-α)·v_PIC + α·(v_old + dv)
            p.vel = (1.f - flipRatio) * vPIC + flipRatio * (p.vel + dv);
        }
    }
}

// ── 5. 更新粒子密度（标记流体格 + 计数）──
void FluidSimulator::updateParticleDensity() {
    std::fill(grid.cellType.begin(), grid.cellType.end(), 0);          // 全标为 air
    std::fill(grid.particleDensity.begin(), grid.particleDensity.end(), 0.f);

    for (auto& p : particles) {
        int i = int(std::floor(p.pos.x() / grid.h));
        int j = int(std::floor(p.pos.y() / grid.h));
        int k = int(std::floor(p.pos.z() / grid.h));
        if (i >= 0 && i < grid.nx && j >= 0 && j < grid.ny && k >= 0 && k < grid.nz) {
            int idx = grid.cIdx(i, j, k);
            grid.cellType[idx]        = 1;  // fluid
            grid.particleDensity[idx] += 1.f;
        }
    }

    // 若 restDensity 尚未设定，用当前平均密度
    if (grid.restDensity == 0.f) {
        float sum = 0.f;
        int   cnt = 0;
        for (int i = 0; i < grid.nx; i++)
            for (int j = 0; j < grid.ny; j++)
                for (int k = 0; k < grid.nz; k++) {
                    int idx = grid.cIdx(i, j, k);
                    if (grid.cellType[idx] == 1) {
                        sum += grid.particleDensity[idx];
                        cnt++;
                    }
                }
        if (cnt > 0) grid.restDensity = sum / cnt;
    }
}

// ── 6. Gauss-Seidel 压力求解 ──
void FluidSimulator::solveIncompressibility(float sdt) {
    if (grid.restDensity == 0.f) return;

    const float rho  = grid.restDensity;
    const float h    = grid.h;
    const float h2   = h * h;
    const float scale = sdt / (rho * h);  // Δu = scale * Δp

    // 在每次迭代前清零压力是一个选择; 这里用累积压力
    for (int iter = 0; iter < numPressureIters; iter++) {
        for (int i = 1; i < grid.nx-1; i++)
            for (int j = 1; j < grid.ny-1; j++)
                for (int k = 1; k < grid.nz-1; k++) {
                    int cIdx = grid.cIdx(i, j, k);
                    if (grid.cellType[cIdx] != 1) continue;   // 只处理流体格

                    // ── 计算散度 ∇·u ──
                    float uLeft = grid.u[grid.uIdx(i, j, k)];
                    float uRight = grid.u[grid.uIdx(i + 1, j, k)];
                    float vBottom = grid.v[grid.vIdx(i, j, k)];
                    float vTop = grid.v[grid.vIdx(i, j + 1, k)];
                    float wBack = grid.w[grid.wIdx(i, j, k)];
                    float wFront = grid.w[grid.wIdx(i, j, k + 1)];

                    float div = (uRight - uLeft + vTop - vBottom + wFront - wBack) / h;

                    // ── 密度漂移补偿 ──
                    if (compensateDrift) {
                        float density = grid.particleDensity[cIdx];
                        if (density > 0.f)
                            div -= (density / rho - 1.f) ;
                    }

                    // ── SOR 压力修正 ──
                    //   dp = -ω · (ρ·h²) / (6·sdt) · div
                    //   推导: 一个压力修正 dp 会在6个面上产生速度变化 Δu = sdt·dp/(ρ·h)
                    //   散度变化 Δdiv = -6·sdt·dp/(ρ·h²), 令 Δdiv = -div, 得 dp = ρ·h²·div/(6·sdt)
                    float dp = -overRelaxation * (rho * h2) / (6.f * sdt) * div;

                    grid.p[cIdx] += dp;

                    // ── 修正相邻面的速度 ──
                    //  dp < 0 时（div>0），将流体拉入格内：
                    //   左面加速 (流入↑), 右面减速 (流出↓), 以此类推
                    float velCorr = scale * dp;
                    grid.u[grid.uIdx(i, j, k)]     -= velCorr;
                    grid.u[grid.uIdx(i + 1, j, k)] += velCorr;
                    grid.v[grid.vIdx(i, j, k)]     -= velCorr;
                    grid.v[grid.vIdx(i, j + 1, k)] += velCorr;
                    grid.w[grid.wIdx(i, j, k)]     -= velCorr;
                    grid.w[grid.wIdx(i, j, k + 1)] += velCorr;
                }
    }
}

} // namespace VCX::Labs::Fluid
