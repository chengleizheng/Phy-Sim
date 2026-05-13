#include "FEMIntegrator.h"

namespace VCX::Labs::FEM {

void FEMIntegrator::ComputeElementForces(
    const Eigen::Vector3f & x0,
    const Eigen::Vector3f & x1,
    const Eigen::Vector3f & x2,
    const Eigen::Vector3f & x3,
    const Eigen::Matrix3f & DmInv,
    float                   restVol,
    Eigen::Vector3f         out[4]) const
{
    // F = [x10, x20, x30] * Dm^{-1}
    Eigen::Matrix3f Ds;
    Ds.col(0) = x1 - x0;
    Ds.col(1) = x2 - x0;
    Ds.col(2) = x3 - x0;
    Eigen::Matrix3f F = Ds * DmInv;

    // G = 1/2 * (F^T * F - I)
    Eigen::Matrix3f G = 0.5f * (F.transpose() * F - Eigen::Matrix3f::Identity());

    // S = 2*mu*G + lambda*trace(G)*I  (StVK second Piola-Kirchhoff stress)
    const float trG = G.trace();
    Eigen::Matrix3f S = 2.0f * material.mu * G
                      + material.lambda * trG * Eigen::Matrix3f::Identity();

    // P = F * S  (first Piola-Kirchhoff stress)
    Eigen::Matrix3f P = F * S;

    // [f1, f2, f3] = -restVol * P * Dm^{-T}
    Eigen::Matrix3f H = -restVol * P * DmInv.transpose();

    out[1] = H.col(0);
    out[2] = H.col(1);
    out[3] = H.col(2);

    // f0 = -f1 - f2 - f3
    out[0] = -(out[1] + out[2] + out[3]);
}

void FEMIntegrator::ComputeAllForces(
    const TetMesh                & mesh,
    std::vector<Eigen::Vector3f> & outForces) const
{
    outForces.assign(mesh.NumVertices(), Eigen::Vector3f::Zero());

    for (int e = 0; e < mesh.NumTets(); ++e) {
        const Eigen::Vector4i & tv = mesh.tets[e];

        Eigen::Vector3f elemForces[4];
        ComputeElementForces(
            mesh.positions[tv[0]],
            mesh.positions[tv[1]],
            mesh.positions[tv[2]],
            mesh.positions[tv[3]],
            mesh.DmInv[e],
            mesh.restVolume[e],
            elemForces);

        for (int i = 0; i < 4; ++i) {
            outForces[tv[i]] += elemForces[i];
        }
    }
}

} // namespace VCX::Labs::FEM
