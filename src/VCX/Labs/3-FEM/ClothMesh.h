#pragma once

#include <vector>
#include <Eigen/Dense>

namespace VCX::Labs::FEM {

struct ClothMesh {
    // 2D parameter-space coordinates (u,v) for rest configuration
    std::vector<Eigen::Vector2f> restUV;
    // 3D deformed positions
    std::vector<Eigen::Vector3f> positions;
    std::vector<Eigen::Vector3f> restPositions;
    std::vector<Eigen::Vector3f> velocities;
    std::vector<float>           masses;
    std::vector<bool>            pinned;

    // Each triangle: 3 vertex indices
    std::vector<Eigen::Vector3i> triangles;

    // Per-triangle: Dm^{-1} (2×2) and reference area |det(Dm)|/2
    std::vector<Eigen::Matrix2f> DmInv;
    std::vector<float>           restArea;

    void BuildGrid(int nx, int ny,
                   float width, float height,
                   float totalMass,
                   float initialHeight = 0.0f);

    void FixCorner(float thresholdU, float thresholdV);

    int NumVertices()  const { return (int) positions.size(); }
    int NumTriangles() const { return (int) triangles.size(); }
};

} // namespace VCX::Labs::FEM
