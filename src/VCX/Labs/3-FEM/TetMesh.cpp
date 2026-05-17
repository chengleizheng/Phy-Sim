#include "TetMesh.h"
#include <map>
#include <tuple>
#include <set>
#include <cmath>



namespace VCX::Labs::FEM {

void TetMesh::BuildBeam(
    int nx, int ny, int nz,
    const Eigen::Vector3f & origin,
    const Eigen::Vector3f & size,
    float totalMass)
{
    const float dx = size.x() / float(nx);
    const float dy = size.y() / float(ny);
    const float dz = size.z() / float(nz);

    const int nvx = nx + 1;
    const int nvy = ny + 1;
    const int nvz = nz + 1;
    //这里是因为点数总比边数多1

    const int numVerts = nvx * nvy * nvz;
    //计算总共的节点数
    const int numTets  = nx * ny * nz * 6;
    //四面体的总个数

    restPositions.resize(numVerts);
    positions.resize(numVerts);
    velocities.assign(numVerts, Eigen::Vector3f::Zero());
    masses.assign(numVerts, totalMass / float(numVerts));
    fixed.assign(numVerts, false);

    for (int k = 0; k < nvz; ++k) {
        for (int j = 0; j < nvy; ++j) {
            for (int i = 0; i < nvx; ++i) {
                int idx = k * nvy * nvx + j * nvx + i;
                Eigen::Vector3f p(
                    origin.x() + i * dx,
                    origin.y() + j * dy,
                    origin.z() + k * dz);
                restPositions[idx] = p;
                positions[idx]     = p;
            }
        }
    }
    //因为初始原因，restPositions和positions都一样，利用三层循环将二者初始化

    tets.resize(numTets);
    DmInv.resize(numTets);
    restVolume.resize(numTets);

    auto V = [nvx, nvy](int i, int j, int k) -> int {
        return k * nvy * nvx + j * nvx + i;
    };
    //相当于将一个三位数组一维化

    const float cellVol = dx * dy * dz / 6.0f;
    //每个四面体的体积
    int tetIdx = 0;

    for (int k = 0; k < nz; ++k) {
        for (int j = 0; j < ny; ++j) {
            for (int i = 0; i < nx; ++i) {
                const int v000 = V(i,   j,   k);
                const int v100 = V(i+1, j,   k);
                const int v110 = V(i+1, j+1, k);
                const int v010 = V(i,   j+1, k);
                const int v001 = V(i,   j,   k+1);
                const int v101 = V(i+1, j,   k+1);
                const int v111 = V(i+1, j+1, k+1);
                const int v011 = V(i,   j+1, k+1);

                // Kuhn decomposition: 6 tets sharing diagonal v000→v111
                // All have positive volume = dx*dy*dz
                const Eigen::Vector4i cellTets[6] = {
                    { v000, v100, v110, v111 },
                    { v000, v100, v111, v101 },
                    { v000, v010, v111, v110 },
                    { v000, v010, v011, v111 },
                    { v000, v001, v101, v111 },
                    { v000, v001, v111, v011 },
                };

                for (int t = 0; t < 6; ++t) {
                    const Eigen::Vector4i tv = cellTets[t];
                    //将一个正方体内的四个顶点存入四个顶点数组
                    //tv[0~5]分别对应四个顶点的四元数
                    tets[tetIdx] = tv;
                    //tetIdx 记录四面体编号
                    //tets[i] 记录第i个四面体对应的四个点标号

                    const Eigen::Vector3f & X0 = restPositions[tv[0]];
                    const Eigen::Vector3f & X1 = restPositions[tv[1]];
                    const Eigen::Vector3f & X2 = restPositions[tv[2]];
                    const Eigen::Vector3f & X3 = restPositions[tv[3]];

                    Eigen::Matrix3f Dm;
                    Dm.col(0) = X1 - X0;
                    Dm.col(1) = X2 - X0;
                    Dm.col(2) = X3 - X0;

                    DmInv[tetIdx]     = Dm.inverse();
                    //预处理数据，把四面体由一个点引发的三条边所对应的张量先求逆，以便下一步处理
                    restVolume[tetIdx] = cellVol;
                    ++tetIdx;
                }
            }
        }
    }
}

void TetMesh::ExtractSurfaceFaces() {
    surfaceFaces.clear();

    std::map<std::tuple<int, int, int>, int> faceCount;

    for (const auto & tv : tets) {
        int v0 = tv[0], v1 = tv[1], v2 = tv[2], v3 = tv[3];

        auto addFace = [&](int a, int b, int c) {
            int arr[3] = { a, b, c };
            if (arr[0] > arr[1]) std::swap(arr[0], arr[1]);
            if (arr[1] > arr[2]) std::swap(arr[1], arr[2]);
            if (arr[0] > arr[1]) std::swap(arr[0], arr[1]);
            faceCount[{ arr[0], arr[1], arr[2] }]++;
        };

        addFace(v0, v1, v2);
        addFace(v0, v1, v3);
        addFace(v0, v2, v3);
        addFace(v1, v2, v3);
    }

    // Second pass: collect surface faces with consistent outward-facing winding.
    // For a positively-oriented tet, cross(b-a, c-a) points INWARD (toward the
    // 4th vertex). For a boundary face we flip it so the normal points outward.
    for (const auto & tv : tets) {
        int v[4] = { tv[0], tv[1], tv[2], tv[3] };

        // Four faces of the tet, each paired with its opposite (4th) vertex
        const int faceVerts[4][3] = {
            { v[0], v[1], v[2] }, // opposite v[3]
            { v[0], v[1], v[3] }, // opposite v[2]
            { v[0], v[2], v[3] }, // opposite v[1]
            { v[1], v[2], v[3] }, // opposite v[0]
        };
        const int opposite[4] = { v[3], v[2], v[1], v[0] };

        for (int fi = 0; fi < 4; ++fi) {
            int a = faceVerts[fi][0], b = faceVerts[fi][1], c = faceVerts[fi][2];

            // Sort for map lookup
            int arr[3] = { a, b, c };
            if (arr[0] > arr[1]) std::swap(arr[0], arr[1]);
            if (arr[1] > arr[2]) std::swap(arr[1], arr[2]);
            if (arr[0] > arr[1]) std::swap(arr[0], arr[1]);

            if (faceCount[{ arr[0], arr[1], arr[2] }] == 1) {
                // Face normal from the tet's winding
                const Eigen::Vector3f & pa = restPositions[a];
                const Eigen::Vector3f & pb = restPositions[b];
                const Eigen::Vector3f & pc = restPositions[c];
                Eigen::Vector3f fn = (pb - pa).cross(pc - pa);
                // Vector from face centroid toward the opposite vertex
                Eigen::Vector3f toInner = restPositions[opposite[fi]]
                                        - (pa + pb + pc) / 3.0f;
                // If dot > 0, the normal points inward → flip winding
                if (fn.dot(toInner) > 0) {
                    surfaceFaces.push_back({ a, c, b }); // swapped → outward
                } else {
                    surfaceFaces.push_back({ a, b, c }); // already outward
                }
            }
        }
    }
}

void TetMesh::FixTopFace(float thresholdY) {
    for (int i = 0; i < NumVertices(); ++i) {
        if (restPositions[i].y() > thresholdY - 1e-5f) {
            fixed[i] = true;
        }
    }
}

} // namespace VCX::Labs::FEM
