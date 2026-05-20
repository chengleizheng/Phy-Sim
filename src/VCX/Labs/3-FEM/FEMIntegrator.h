#pragma once

#include <Eigen/Dense>
#include "TetMesh.h"

namespace VCX::Labs::FEM {

enum class MaterialModel {
    StVK,
    NeoHookean,
    Corotated
};

struct StVKMaterial {
    float lambda = 300.0f;
    float mu     = 50.0f;
};

class FEMIntegrator {
public:
    StVKMaterial  material;
    MaterialModel materialModel = MaterialModel::StVK;

    void ComputeElementForces(
        const Eigen::Vector3f & x0,
        const Eigen::Vector3f & x1,
        const Eigen::Vector3f & x2,
        const Eigen::Vector3f & x3,
        const Eigen::Matrix3f & DmInv,
        float                   restVol,
        Eigen::Vector3f         out[4]) const;

    void ComputeAllForces(
        const TetMesh                & mesh,
        std::vector<Eigen::Vector3f> & outForces) const;

    // Compute 12×12 element tangent stiffness K_e = -∂f/∂x for StVK.
    // K_e.block<3,3>(3*i, 3*j) = -∂f_i / ∂x_j
    void ComputeElementTangentStiffness(
        const Eigen::Vector3f            & x0,
        const Eigen::Vector3f            & x1,
        const Eigen::Vector3f            & x2,
        const Eigen::Vector3f            & x3,
        const Eigen::Matrix3f            & DmInv,
        float                              restVol,
        Eigen::Matrix<float, 12, 12>     & Ke) const;
};

} // namespace VCX::Labs::FEM
