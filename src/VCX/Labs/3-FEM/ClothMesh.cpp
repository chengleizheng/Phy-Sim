#include "ClothMesh.h"

namespace VCX::Labs::FEM {

void ClothMesh::BuildGrid(
    int nx, int ny,
    float width, float height,
    float totalMass,
    float initialHeight)
{
    const float dx = width  / float(nx);
    const float dy = height / float(ny);

    const int nvx = nx + 1;
    const int nvy = ny + 1;
    const int numVerts = nvx * nvy;
    const int numTris  = nx * ny * 2;

    restUV.resize(numVerts);
    restPositions.resize(numVerts);
    positions.resize(numVerts);
    velocities.assign(numVerts, Eigen::Vector3f::Zero());
    masses.assign(numVerts, totalMass / float(numVerts));
    pinned.assign(numVerts, false);

    // Place vertices in XZ plane centred at origin, Y = initialHeight
    for (int j = 0; j < nvy; ++j) {
        for (int i = 0; i < nvx; ++i) {
            int idx = j * nvx + i;
            float u = i * dx;
            float v = j * dy;
            restUV[idx]      = Eigen::Vector2f(u, v);
            Eigen::Vector3f p(u - width * 0.5f, initialHeight, v - height * 0.5f);
            restPositions[idx] = p;
            positions[idx]     = p;
        }
    }

    auto idx = [nvx](int i, int j) -> int {
        return j * nvx + i;
    };

    triangles.resize(numTris);
    DmInv.resize(numTris);
    restArea.resize(numTris);

    const float triArea = dx * dy * 0.5f;
    int tri = 0;

    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            int v00 = idx(i,   j);
            int v10 = idx(i+1, j);
            int v11 = idx(i+1, j+1);
            int v01 = idx(i,   j+1);

            // Triangle 1: (i,j)-(i+1,j+1)-(i+1,j)  (upper-right)
            {
                triangles[tri] = Eigen::Vector3i(v00, v11, v10);

                const Eigen::Vector2f &u0 = restUV[v00];
                const Eigen::Vector2f &u1 = restUV[v11];
                const Eigen::Vector2f &u2 = restUV[v10];

                Eigen::Matrix2f Dm;
                Dm.col(0) = u1 - u0; // (dx, dy)
                Dm.col(1) = u2 - u0; // (dx, 0)
                DmInv[tri]   = Dm.inverse();
                restArea[tri] = triArea;
                ++tri;
            }

            // Triangle 2: (i,j)-(i,j+1)-(i+1,j+1)  (lower-left)
            {
                triangles[tri] = Eigen::Vector3i(v00, v01, v11);

                const Eigen::Vector2f &u0 = restUV[v00];
                const Eigen::Vector2f &u1 = restUV[v01];
                const Eigen::Vector2f &u2 = restUV[v11];

                Eigen::Matrix2f Dm;
                Dm.col(0) = u1 - u0; // (0, dy)
                Dm.col(1) = u2 - u0; // (dx, dy)
                DmInv[tri]   = Dm.inverse();
                restArea[tri] = triArea;
                ++tri;
            }
        }
    }
}

void ClothMesh::FixCorner(float thresholdU, float thresholdV) {
    // Find max U for the second corner
    float maxU = 0.0f;
    for (const auto & uv : restUV) {
        if (uv.x() > maxU) maxU = uv.x();
    }
    for (int i = 0; i < NumVertices(); ++i) {
        float u = restUV[i].x();
        float v = restUV[i].y();
        if (v > thresholdV - 1e-5f) {
            if (u < thresholdU + 1e-5f || u > maxU - thresholdU - 1e-5f) {
                pinned[i] = true;
            }
        }
    }
}

} // namespace VCX::Labs::FEM
