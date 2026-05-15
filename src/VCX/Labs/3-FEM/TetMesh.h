#pragma once

#include <vector>
#include <Eigen/Dense>

namespace VCX::Labs::FEM {

struct TetMesh {
    std::vector<Eigen::Vector3f> restPositions; // X: 参考构型顶点
    std::vector<Eigen::Vector3f> positions; // x: 当前变形后顶点
    std::vector<Eigen::Vector3f> velocities;    // v: 顶点速度
    std::vector<float>           masses;    // m: 集中质量 (lumped mass)
    std::vector<bool>            fixed;

    std::vector<Eigen::Vector4i> tets;  // 每个四面体的4个顶点索引

    std::vector<Eigen::Matrix3f> DmInv; // D_m^{-1}
    std::vector<float>           restVolume;    // 参考构型体积 = |det(Dm)|/6

    std::vector<Eigen::Vector3i> surfaceFaces;  // 表面三角面 (用于渲染)

    void BuildBeam(
        int nx, int ny, int nz,
        const Eigen::Vector3f & origin,
        const Eigen::Vector3f & size,
        float totalMass);

    void ExtractSurfaceFaces();
    void FixTopFace(float thresholdY);

    int NumVertices() const { return (int) restPositions.size(); }
    int NumTets()     const { return (int) tets.size(); }
};

} // namespace VCX::Labs::FEM
