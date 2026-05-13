#pragma once

#include <vector>
#include <Eigen/Dense>

namespace VCX::Labs::FEM {

struct TetMesh {
    std::vector<Eigen::Vector3f> restPositions;
    std::vector<Eigen::Vector3f> positions;
    std::vector<Eigen::Vector3f> velocities;
    std::vector<float>           masses;
    std::vector<bool>            fixed;

    std::vector<Eigen::Vector4i> tets;

    std::vector<Eigen::Matrix3f> DmInv;
    std::vector<float>           restVolume;

    std::vector<Eigen::Vector3i> surfaceFaces;

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
