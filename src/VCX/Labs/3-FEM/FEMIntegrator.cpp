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

    Eigen::Matrix3f P;

    if (materialModel == MaterialModel::NeoHookean) {
        // Neo-Hookean: P = mu*(F - F^{-T}) + lambda*ln(J)*F^{-T}
        const float J = F.determinant();
        Eigen::Matrix3f F_inv_T = F.inverse().transpose();
        float logJ = J > 1e-8f ? std::log(J) : 0.0f;
        P = material.mu * (F - F_inv_T) + material.lambda * logJ * F_inv_T;
    } else if (materialModel == MaterialModel::Corotated) {
        // Corotated: polar decompose F = R*S, then P = R * [2*mu*(S-I) + lambda*tr(S-I)*I]
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3f R = svd.matrixU() * svd.matrixV().transpose();
        if (R.determinant() < 0) {
            Eigen::Matrix3f U = svd.matrixU();
            U.col(2) *= -1.0f;
            R = U * svd.matrixV().transpose();
        }
        Eigen::Matrix3f S = R.transpose() * F;
        Eigen::Matrix3f eps = S - Eigen::Matrix3f::Identity();
        P = R * (2.0f * material.mu * eps + material.lambda * eps.trace() * Eigen::Matrix3f::Identity());
    } else {
        // StVK: G = 1/2*(F^T*F - I), S = 2*mu*G + lambda*tr(G)*I, P = F*S
        Eigen::Matrix3f G = 0.5f * (F.transpose() * F - Eigen::Matrix3f::Identity());
        const float trG = G.trace();
        Eigen::Matrix3f S = 2.0f * material.mu * G
                          + material.lambda * trG * Eigen::Matrix3f::Identity();
        P = F * S;
    }

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

void FEMIntegrator::ComputeElementTangentStiffness(
    const Eigen::Vector3f            & x0,
    const Eigen::Vector3f            & x1,
    const Eigen::Vector3f            & x2,
    const Eigen::Vector3f            & x3,
    const Eigen::Matrix3f            & DmInv,
    float                              restVol,
    Eigen::Matrix<float, 12, 12>     & Ke) const
{
    Ke.setZero();

    // Deformation gradient F = Ds * DmInv
    Eigen::Matrix3f Ds;
    Ds.col(0) = x1 - x0;
    Ds.col(1) = x2 - x0;
    Ds.col(2) = x3 - x0;
    Eigen::Matrix3f F   = Ds * DmInv;
    Eigen::Matrix3f DmInvT = DmInv.transpose();

    // StVK: G = 1/2*(F^T*F - I), S = 2mu*G + lambda*tr(G)*I
    Eigen::Matrix3f G   = 0.5f * (F.transpose() * F - Eigen::Matrix3f::Identity());
    float           trG = G.trace();
    Eigen::Matrix3f S   = 2.0f * material.mu * G
                        + material.lambda * trG * Eigen::Matrix3f::Identity();

    // coeff_j selects how vertex j perturbs Ds columns:
    //   Ds.col(0)=x1-x0, Ds.col(1)=x2-x0, Ds.col(2)=x3-x0
    Eigen::Vector3f coeff[4] = {
        Eigen::Vector3f(-1, -1, -1),  // j=0: all 3 cols get -δx
        Eigen::Vector3f( 1,  0,  0),  // j=1: col 0 gets +δx
        Eigen::Vector3f( 0,  1,  0),  // j=2: col 1 gets +δx
        Eigen::Vector3f( 0,  0,  1)   // j=3: col 2 gets +δx
    };

    // w_i selects force component from H = -V0*P*Dm^{-T}:
    //   f_0 = -(H.col0 + H.col1 + H.col2), f_k = H.col(k-1) for k=1,2,3
    Eigen::Vector3f w[4] = {
        Eigen::Vector3f(-1, -1, -1),
        Eigen::Vector3f( 1,  0,  0),
        Eigen::Vector3f( 0,  1,  0),
        Eigen::Vector3f( 0,  0,  1)
    };

    for (int j = 0; j < 4; ++j) {
        // g_j = DmInv^T * coeff_j  (3×1): direction of δF for vertex j
        Eigen::Vector3f gj = DmInvT * coeff[j];

        for (int b = 0; b < 3; ++b) {
            // δF for perturbation e_b at vertex j: δF = e_b * g_j^T
            // Row b = g_j^T, all other rows zero

            // δG = 0.5 * (g_j * F.row(b) + F.row(b)^T * g_j^T)
            Eigen::Matrix3f dG = 0.5f * (gj * F.row(b)
                                       + F.row(b).transpose() * gj.transpose());

            // δS = 2μ*δG + λ*tr(δG)*I
            Eigen::Matrix3f dS = 2.0f * material.mu * dG
                               + material.lambda * dG.trace() * Eigen::Matrix3f::Identity();

            // δP = δF*S + F*δS
            // δF*S = e_b * (g_j^T * S)  — only row b is non-zero
            Eigen::Matrix3f dP = F * dS;
            dP.row(b) += gj.transpose() * S;  // gj^T * S = (1×3)

            // δH = -restVol * δP * Dm^{-T}
            Eigen::Matrix3f dH = -restVol * dP * DmInvT;

            // δf_i = δH * w_i,  K_e.block(3*i, 3*j).col(b) = -δf_i = -dH * w_i
            for (int i = 0; i < 4; ++i) {
                Ke.block<3, 1>(3 * i, 3 * j + b) = -dH * w[i];
            }
        }
    }
}

} // namespace VCX::Labs::FEM
