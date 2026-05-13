#pragma once

#include <Eigen/Dense>
#include "TetMesh.h"

namespace VCX::Labs::FEM {

struct StVKMaterial {
    float lambda = 1000.0f;
    float mu     = 500.0f;
};

class FEMIntegrator {
public:
    StVKMaterial material;

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
};

} // namespace VCX::Labs::FEM
