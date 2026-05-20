#include "ClothFEMIntegrator.h"

namespace VCX::Labs::FEM {

void ClothFEMIntegrator::ComputeElementForces(
    const Eigen::Vector3f & x0,
    const Eigen::Vector3f & x1,
    const Eigen::Vector3f & x2,
    const Eigen::Vector3f & v0,
    const Eigen::Vector3f & v1,
    const Eigen::Vector3f & v2,
    const Eigen::Matrix2f & DmInv,
    float                   restArea,
    Eigen::Vector3f         out[3]) const
{
    for (int i = 0; i < 3; ++i) out[i] = Eigen::Vector3f::Zero();

    // F = Ds * Dm^{-1}  (3×2)
    Eigen::Matrix<float, 3, 2> Ds;
    Ds.col(0) = x1 - x0;
    Ds.col(1) = x2 - x0;
    Eigen::Matrix<float, 3, 2> F = Ds * DmInv;

    // Ḟ = Dv * Dm^{-1}  (3×2) — velocity gradient for rate-dependent damping
    Eigen::Matrix<float, 3, 2> Dv;
    Dv.col(0) = v1 - v0;
    Dv.col(1) = v2 - v0;
    Eigen::Matrix<float, 3, 2> Fdot = Dv * DmInv;

    // Right Cauchy-Green: C = F^T * F  (2×2)
    Eigen::Matrix2f C = F.transpose() * F;
    float J2 = C.determinant();  // J² = det(C) = squared area ratio

    // Guard against degenerate / inverted triangles
    if (J2 < 1e-8f) return;

    // === Elastic PK1 stress P (model-specific) ===
    Eigen::Matrix<float, 3, 2> P;

    if (materialModel == MaterialModel::NeoHookean) {
        float           logJ  = 0.5f * std::log(J2);
        Eigen::Matrix2f C_inv = C.inverse();
        Eigen::Matrix<float, 3, 2> F_Cinv = F * C_inv;
        P = material.mu * (F - F_Cinv)
          + material.lambda * logJ * F_Cinv;

    } else if (materialModel == MaterialModel::Corotated) {
        Eigen::JacobiSVD<Eigen::Matrix<float, 3, 2>> svd(
            F, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix<float, 3, 2> U2 = svd.matrixU().leftCols(2);
        Eigen::Matrix2f V  = svd.matrixV();
        Eigen::Vector2f sigma = svd.singularValues();

        // Strain from singular values directly: σ_i - 1
        Eigen::Vector2f sigmaMinus1 = sigma.array() - 1.0f;
        float trEps = sigmaMinus1.sum();

        // Diagonal stress in principal frame
        Eigen::Vector2f S_diag = 2.0f * material.mu * sigmaMinus1
                               + material.lambda * trEps * Eigen::Vector2f::Ones();

        // P = U * diag(S_diag) * V^T  (thin SVD reconstruction)
        P = U2 * S_diag.asDiagonal() * V.transpose();

    } else { // StVK
        Eigen::Matrix2f G   = 0.5f * (C - Eigen::Matrix2f::Identity());
        float           trG = G.trace();
        Eigen::Matrix2f S   = 2.0f * material.mu * G
                            + material.lambda * trG * Eigen::Matrix2f::Identity();
        P = F * S;
    }

    // === Elastic forces: [f1, f2] = -restArea * P * Dm^{-T} ===
    Eigen::Matrix<float, 3, 2> H = -restArea * P * DmInv.transpose();
    out[1] = H.col(0);
    out[2] = H.col(1);
    out[0] = -(out[1] + out[2]);

    // === Stiffness-proportional Rayleigh damping ===
    // Use StVK-linearized Ṗ as approximation for all models:
    //   Ġ = ½(Ḟ^T F + F^T Ḟ),  Ṡ = 2μ Ġ + λ tr(Ġ) I,  Ṗ = Ḟ S + F Ṡ
    if (material.beta > 0.0f) {
        // StVK G and S (reuse or recompute)
        Eigen::Matrix2f G    = 0.5f * (C - Eigen::Matrix2f::Identity());
        float           trG  = G.trace();
        Eigen::Matrix2f S_stvk = 2.0f * material.mu * G
                               + material.lambda * trG * Eigen::Matrix2f::Identity();

        Eigen::Matrix2f Cdot = Fdot.transpose() * F + F.transpose() * Fdot;
        Eigen::Matrix2f Gdot = 0.5f * Cdot;
        float           trGd = Gdot.trace();
        Eigen::Matrix2f Sdot = 2.0f * material.mu * Gdot
                             + material.lambda * trGd * Eigen::Matrix2f::Identity();

        Eigen::Matrix<float, 3, 2> Pdot = Fdot * S_stvk + F * Sdot;

        Eigen::Matrix<float, 3, 2> Hdamp =
            -restArea * material.beta * Pdot * DmInv.transpose();

        out[1] += Hdamp.col(0);
        out[2] += Hdamp.col(1);
        out[0] -= (Hdamp.col(0) + Hdamp.col(1));
    }
}

void ClothFEMIntegrator::ComputeAllForces(
    const ClothMesh              & mesh,
    std::vector<Eigen::Vector3f> & outForces) const
{
    outForces.assign(mesh.NumVertices(), Eigen::Vector3f::Zero());

    for (int e = 0; e < mesh.NumTriangles(); ++e) {
        const Eigen::Vector3i & tv = mesh.triangles[e];

        Eigen::Vector3f elemForces[3];
        ComputeElementForces(
            mesh.positions[tv[0]],
            mesh.positions[tv[1]],
            mesh.positions[tv[2]],
            mesh.velocities[tv[0]],
            mesh.velocities[tv[1]],
            mesh.velocities[tv[2]],
            mesh.DmInv[e],
            mesh.restArea[e],
            elemForces);

        for (int i = 0; i < 3; ++i)
            outForces[tv[i]] += elemForces[i];
    }
}

} // namespace VCX::Labs::FEM
