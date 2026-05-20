#pragma once

#include <Eigen/Dense>
#include "ClothMesh.h"
#include "FEMIntegrator.h" // for MaterialModel

namespace VCX::Labs::FEM {

struct ClothMaterial {
    // Plane-stress Lamé parameters derived from Young's modulus E and Poisson ratio nu:
    //   mu = E / (2*(1+nu))
    //   lambda = E*nu / (1 - nu*nu)
    float lambda = 659.0f;  // from E=2000, nu=0.3
    float mu     = 769.0f;
    float beta   = 0.001f; // stiffness-proportional Rayleigh damping
};

class ClothFEMIntegrator {
public:
    ClothMaterial  material;
    MaterialModel  materialModel = MaterialModel::StVK;

    // Compute forces for a single triangle element.
    // x0,x1,x2: deformed 3D positions
    // DmInv:    2×2 inverse of reference edge matrix
    // restArea: reference area of triangle = |det(Dm)| / 2
    void ComputeElementForces(
        const Eigen::Vector3f & x0,
        const Eigen::Vector3f & x1,
        const Eigen::Vector3f & x2,
        const Eigen::Vector3f & v0,   // 新增
        const Eigen::Vector3f & v1,   // 新增
        const Eigen::Vector3f & v2, 
        const Eigen::Matrix2f & DmInv,
        float                   restArea,
        Eigen::Vector3f         out[3]) const;

    void ComputeAllForces(
        const ClothMesh              & mesh,
        std::vector<Eigen::Vector3f> & outForces) const;
};

} // namespace VCX::Labs::FEM
