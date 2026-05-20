#include "CaseFEMSoftBody.h"
#include "Labs/Common/ImGuiHelper.h"
#include "Engine/app.h"
#include <set>
#include <algorithm>

static glm::vec3 eigen2glm(const Eigen::Vector3f & v) {
    return glm::vec3(v.x(), v.y(), v.z());
}

static Eigen::Vector3f glm2eigen(const glm::vec3 & v) {
    return Eigen::Vector3f(v.x, v.y, v.z);
}

namespace VCX::Labs::FEM {

CaseFEMSoftBody::CaseFEMSoftBody() :
    _program(
        Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"),
                                    Engine::GL::SharedShader("assets/shaders/flat.frag") })),
    _litProgram(
        Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/lit_mesh.vert"),
                                    Engine::GL::SharedShader("assets/shaders/lit_mesh.frag") })),
    _surfaceItem(
        Engine::GL::VertexLayout()
            .Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0)
            .Add<glm::vec3>("normal",   Engine::GL::DrawFrequency::Stream, 1),
        Engine::GL::PrimitiveType::Triangles),
    _wireItem(
        Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0),
        Engine::GL::PrimitiveType::Lines)
{
    _cameraManager.AutoRotate = false;
    _cameraManager.Save(_camera);

    _forceManager.ForceScale = 0.05f;

    RebuildMesh();
}

void CaseFEMSoftBody::RebuildMesh() {
    _mesh.BuildBeam(_gridResX, _gridResY, _gridResZ,
                    glm2eigen(_beamOrigin),
                    glm2eigen(_beamSize),
                    _totalMass);

    if (_fixTopFace) {
        float topY = _beamOrigin.y + _beamSize.y;
        _mesh.FixTopFace(topY);
    }

    _mesh.ExtractSurfaceFaces();

    // Build triangle index buffer
    {
        std::vector<std::uint32_t> triIdx;
        triIdx.reserve(_mesh.surfaceFaces.size() * 3);
        for (const auto & f : _mesh.surfaceFaces) {
            triIdx.push_back(f[0]);
            triIdx.push_back(f[1]);
            triIdx.push_back(f[2]);
        }
        _surfaceItem.UpdateElementBuffer(triIdx);
    }

    // Build wireframe line index buffer (deduplicated edges)
    {
        std::set<std::pair<int, int>> edgeSet;
        for (const auto & f : _mesh.surfaceFaces) {
            for (int e = 0; e < 3; ++e) {
                int a = f[e], b = f[(e + 1) % 3];
                if (a > b) std::swap(a, b);
                edgeSet.insert({ a, b });
            }
        }
        std::vector<std::uint32_t> lineIdx;
        lineIdx.reserve(edgeSet.size() * 2);
        for (const auto & e : edgeSet) {
            lineIdx.push_back(e.first);
            lineIdx.push_back(e.second);
        }
        _wireItem.UpdateElementBuffer(lineIdx);
    }
}

void CaseFEMSoftBody::ResetSimulation() {
    for (int i = 0; i < _mesh.NumVertices(); ++i) {
        _mesh.positions[i]  = _mesh.restPositions[i];
        _mesh.velocities[i] = Eigen::Vector3f::Zero();
    }
}

void CaseFEMSoftBody::OnSetupPropsUI() {
    const ImGuiIO & io = ImGui::GetIO();
    const bool altHeld = io.KeyAlt || ImGui::IsKeyDown(ImGuiKey_LeftAlt) || ImGui::IsKeyDown(ImGuiKey_RightAlt);
    const bool altSpaceHeld = altHeld && ImGui::IsKeyDown(ImGuiKey_Space);
    ImGui::TextColored(ImVec4(0.5f, 1.0f, 0.5f, 1.0f),
            altSpaceHeld ? "[ALT+SPACE] held - lifting!" : "Hold [ALT+SPACE] to lift");
    if (ImGui::CollapsingHeader("Appearance", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("Lighting", &_useLighting);
        ImGui::ColorEdit3("Surface Color", glm::value_ptr(_surfaceColor));
        if (_useLighting) {
            ImGui::SliderFloat("Light Intensity", &_lightIntensity, 0.1f, 3.0f);
            ImGui::SliderFloat("Ambient", &_ambientScale, 0.0f, 0.5f);
            ImGui::SliderFloat("Shininess", &_shininess, 1.0f, 256.0f);
            ImGui::Checkbox("Flat Shading", &_flatShading);
        }
        ImGui::Checkbox("Show Wireframe", &_showWireframe);
    }

    if (ImGui::CollapsingHeader("Physics", ImGuiTreeNodeFlags_DefaultOpen)) {
        // Material model selector
        {
            int currentModel = static_cast<int>(_integrator.materialModel);
            if (ImGui::Combo("Model", &currentModel, "StVK\0Neo-Hookean\0Corotated\0")) {
                _integrator.materialModel = static_cast<MaterialModel>(currentModel);
            }
        }
        bool changed = false;
        changed |= ImGui::SliderFloat("Lambda", &_lambda, 100.0f, 1000.0f);
        changed |= ImGui::SliderFloat("Mu", &_mu, 10.0f, 200.0f);
        if (changed) {
            _integrator.material.lambda = _lambda;
            _integrator.material.mu     = _mu;
        }
        ImGui::SliderFloat3("Gravity", glm::value_ptr(_gravity), -20.0f, 0.0f);
    }

    if (ImGui::CollapsingHeader("Mesh", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SliderInt("Grid X", &_gridResX, 3, 20);
        ImGui::SliderInt("Grid Y", &_gridResY, 2, 10);
        ImGui::SliderInt("Grid Z", &_gridResZ, 2, 10);
        if (ImGui::Button("Rebuild Mesh")) {
            _needsRebuild = true;
            //根据新改的尺寸rebuild
        }
        /*ImGui::SameLine();
        if (ImGui::Checkbox("Fix Top Face", &_fixTopFace)) {
            _needsRebuild = true;
        }
        */
    }

    if (ImGui::CollapsingHeader("Interaction", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SliderFloat("Lift Force", &_liftForce, 10.0f, 200.0f);
    }

    if (ImGui::CollapsingHeader("Simulation", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("Implicit", &_useImplicit);
        ImGui::SameLine();
        ImGui::TextDisabled("(?)");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Backward Euler with Newton + CG.\nStable with fewer substeps but slower per step.");

        if (_useImplicit) {
            ImGui::SliderInt("Newton Iters", &_maxNewtonIters, 1, 10);
            ImGui::SliderInt("Max CG Iters", &_maxCGIters, 50, 500);
            ImGui::SliderFloat("CG Tolerance", &_cgTolerance, 1e-6f, 1e-2f, "%.6f");
            ImGui::TextColored(_cgConverged
                ? ImVec4(0.3f, 1.0f, 0.3f, 1.0f)
                : ImVec4(1.0f, 0.3f, 0.3f, 1.0f),
                _cgConverged ? "CG: converged" : "CG: NOT converged");
        } else {
            ImGui::SliderInt("Substeps", &_numSubsteps, 1, 200);
        }
        if (ImGui::Button("Reset")) {
            ResetSimulation();
        }
    }
    ImGui::Spacing();
}

void CaseFEMSoftBody::Advance(float dt) {
    if (dt <= 0.0f) return;

    dt = std::min(dt, 1.0f / 30.0f);

    if (_useImplicit) {
        // Implicit handles its own substeps via Newton-CG
        AdvanceImplicit(dt);
    } else {
        const float subDt = dt / float(_numSubsteps);
        const Eigen::Vector3f grav3 = glm2eigen(_gravity);
        const ImGuiIO & io = ImGui::GetIO();
        const bool altHeld = io.KeyAlt || ImGui::IsKeyDown(ImGuiKey_LeftAlt) || ImGui::IsKeyDown(ImGuiKey_RightAlt);
        const bool spaceHeld = altHeld && ImGui::IsKeyDown(ImGuiKey_Space);
        for (int step = 0; step < _numSubsteps; ++step) {
            AdvanceExplicit(subDt, grav3, spaceHeld);
        }
    }
}

void CaseFEMSoftBody::AdvanceExplicit(
    float subDt, const Eigen::Vector3f & grav3, bool spaceHeld)
{
    std::vector<Eigen::Vector3f> forces;
    _integrator.ComputeAllForces(_mesh, forces);

    for (int i = 0; i < _mesh.NumVertices(); ++i) {
        if (_mesh.fixed[i]) {
            _mesh.positions[i]  = _mesh.restPositions[i];
            _mesh.velocities[i] = Eigen::Vector3f::Zero();
            continue;
        }

        forces[i] += _mesh.masses[i] * grav3;
        if (spaceHeld) {
            forces[i] += Eigen::Vector3f(0.0f, _liftForce * _mesh.masses[i], 0.0f);
        }

        _mesh.velocities[i] += subDt * forces[i] / _mesh.masses[i];

        const float dampFactor = std::exp(-_damping * subDt);
        _mesh.velocities[i] *= dampFactor;

        _mesh.positions[i] += subDt * _mesh.velocities[i];

        if (_mesh.positions[i].y() < _floorY) {
            _mesh.positions[i].y() = _floorY;
            if (_mesh.velocities[i].y() < 0.0f) {
                _mesh.velocities[i].y() = 0.0f;
            }
        }
    }
}

void CaseFEMSoftBody::AdvanceImplicit(float dt) {
    const int nv    = _mesh.NumVertices();
    const int ndof  = nv * 3;
    const float dt2 = dt * dt;
    const Eigen::Vector3f grav3 = glm2eigen(_gravity);
    const ImGuiIO & io = ImGui::GetIO();
    const bool altHeld = io.KeyAlt || ImGui::IsKeyDown(ImGuiKey_LeftAlt) || ImGui::IsKeyDown(ImGuiKey_RightAlt);
    const bool spaceHeld = altHeld && ImGui::IsKeyDown(ImGuiKey_Space);

    // Save state before implicit step
    std::vector<Eigen::Vector3f> posOld = _mesh.positions;
    std::vector<Eigen::Vector3f> velOld = _mesh.velocities;

    // Prediction: x = x_n + dt * v_n  (explicit predictor)
    for (int i = 0; i < nv; ++i) {
        if (!_mesh.fixed[i]) {
            _mesh.positions[i] += dt * _mesh.velocities[i];
        }
    }

    // Helper: assemble triplets for A = M + dt²*K, skipping fixed DOFs
    auto buildTriplets = [&](std::vector<Eigen::Triplet<float>> & trips) {
        trips.clear();
        trips.reserve(nv * 3 + _mesh.NumTets() * 144);

        // Mass diagonal
        for (int i = 0; i < nv; ++i) {
            if (_mesh.fixed[i]) continue;
            float m = _mesh.masses[i];
            for (int d = 0; d < 3; ++d) {
                int dof = 3 * i + d;
                trips.push_back({dof, dof, m});
            }
        }

        // Tangent stiffness: Δt² * K_e per tet
        for (int e = 0; e < _mesh.NumTets(); ++e) {
            const Eigen::Vector4i & tv = _mesh.tets[e];

            Eigen::Matrix<float, 12, 12> Ke;
            _integrator.ComputeElementTangentStiffness(
                _mesh.positions[tv[0]], _mesh.positions[tv[1]],
                _mesh.positions[tv[2]], _mesh.positions[tv[3]],
                _mesh.DmInv[e], _mesh.restVolume[e], Ke);

            for (int i = 0; i < 4; ++i) {
                int gi = tv[i];
                if (_mesh.fixed[gi]) continue;
                for (int j = 0; j < 4; ++j) {
                    int gj = tv[j];
                    if (_mesh.fixed[gj]) continue;
                    for (int a = 0; a < 3; ++a) {
                        for (int b = 0; b < 3; ++b) {
                            float val = dt2 * Ke(3 * i + a, 3 * j + b);
                            if (std::abs(val) > 1e-12f) {
                                trips.push_back({3 * gi + a, 3 * gj + b, val});
                            }
                        }
                    }
                }
            }
        }
    };

    // Newton loop
    std::vector<Eigen::Triplet<float>> triplets;
    Eigen::SparseMatrix<float> A;
    Eigen::ConjugateGradient<Eigen::SparseMatrix<float>,
                             Eigen::Lower | Eigen::Upper,
                             Eigen::DiagonalPreconditioner<float>> cg;
    cg.setMaxIterations(_maxCGIters);
    cg.setTolerance(_cgTolerance);

    bool cgFailed = false;

    for (int newton = 0; newton < _maxNewtonIters && !cgFailed; ++newton) {
        // Compute elastic forces at current positions
        std::vector<Eigen::Vector3f> f_elastic;
        _integrator.ComputeAllForces(_mesh, f_elastic);

        // Assemble system matrix
        buildTriplets(triplets);
        // Identity for fixed DOFs (so A is non-singular)
        for (int i = 0; i < nv; ++i) {
            if (!_mesh.fixed[i]) continue;
            for (int d = 0; d < 3; ++d)
                triplets.push_back({3 * i + d, 3 * i + d, 1.0f});
        }
        A.resize(ndof, ndof);
        A.setFromTriplets(triplets.begin(), triplets.end());

        // Build RHS: r = dt² * f_total - M * (x_cur - x_n - dt * v_n)
        Eigen::VectorXf rhs(ndof);
        rhs.setZero();
        for (int i = 0; i < nv; ++i) {
            if (_mesh.fixed[i]) continue;
            Eigen::Vector3f f_total = f_elastic[i]
                                    + _mesh.masses[i] * grav3;
            if (spaceHeld) {
                f_total += Eigen::Vector3f(0.0f, _liftForce * _mesh.masses[i], 0.0f);
            }
            // Inertial residual: M * (x_cur - x_n - dt * v_n)
            Eigen::Vector3f r_inertial = _mesh.masses[i]
                * (_mesh.positions[i] - posOld[i] - dt * velOld[i]);
            for (int d = 0; d < 3; ++d) {
                rhs[3 * i + d] = dt2 * f_total[d] - r_inertial[d];
            }
        }

        // Solve A * Δx = rhs with CG + diagonal preconditioner
        cg.compute(A);
        if (cg.info() != Eigen::Success) {
            cgFailed = true;
            break;
        }
        Eigen::VectorXf dx = cg.solve(rhs);
        if (cg.info() == Eigen::Success) {
            _cgConverged = true;
        } else if (cg.info() == Eigen::NoConvergence) {
            _cgConverged = false;
        } else {
            cgFailed = true;
            break;
        }

        // Clamp per-vertex displacement to prevent explosions
        const float maxDx = 0.5f; // max 0.5m per Newton iteration
        for (int i = 0; i < nv; ++i) {
            if (_mesh.fixed[i]) continue;
            Eigen::Vector3f dxi(dx[3 * i], dx[3 * i + 1], dx[3 * i + 2]);
            float len = dxi.norm();
            if (len > maxDx) dxi *= maxDx / len;
            for (int d = 0; d < 3; ++d) dx[3 * i + d] = dxi[d];
        }

        // Update positions
        for (int i = 0; i < nv; ++i) {
            if (_mesh.fixed[i]) continue;
            for (int d = 0; d < 3; ++d) {
                _mesh.positions[i][d] += dx[3 * i + d];
            }
        }
    }

    // If CG failed, fall back to explicit
    if (cgFailed) {
        _mesh.positions = posOld;
        _mesh.velocities = velOld;
        AdvanceExplicit(dt, grav3, spaceHeld);
        return;
    }

    // Compute velocity from total displacement
    for (int i = 0; i < nv; ++i) {
        if (_mesh.fixed[i]) {
            _mesh.positions[i]  = _mesh.restPositions[i];
            _mesh.velocities[i] = Eigen::Vector3f::Zero();
            continue;
        }
        _mesh.velocities[i] = (_mesh.positions[i] - posOld[i]) / dt;

        // Apply damping to velocity (exponential)
        const float dampFactor = std::exp(-_damping * dt);
        _mesh.velocities[i] *= dampFactor;

        // Floor collision
        if (_mesh.positions[i].y() < _floorY) {
            _mesh.positions[i].y() = _floorY;
            if (_mesh.velocities[i].y() < 0.0f) {
                _mesh.velocities[i].y() = 0.0f;
            }
        }

        // NaN guard
        if (!_mesh.positions[i].allFinite() || !_mesh.velocities[i].allFinite()) {
            _mesh.positions[i]  = _mesh.restPositions[i];
            _mesh.velocities[i] = Eigen::Vector3f::Zero();
        }
    }
}
void CaseFEMSoftBody::ApplyMouseForce() {
    std::vector<glm::vec3> candidates;
    candidates.reserve(_mesh.NumVertices());
    for (int i = 0; i < _mesh.NumVertices(); ++i) {
        candidates.push_back(eigen2glm(_mesh.positions[i]));
    }

    auto [force, idx] = _forceManager.getForce(candidates);

    if (glm::length(force) > 1e-6f && idx >= 0 && !_mesh.fixed[idx]) {
        _mesh.velocities[idx] += glm2eigen(force) / _mesh.masses[idx];
    }
}

Common::CaseRenderResult CaseFEMSoftBody::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
    if (_needsRebuild) {
        RebuildMesh();
        _needsRebuild = false;
    }

    ApplyMouseForce();
    Advance(Engine::GetDeltaTime());

    _frame.Resize(desiredSize);

    _cameraManager.Update(_camera);

    const auto projMat = _camera.GetProjectionMatrix(float(desiredSize.first) / float(desiredSize.second));
    const auto viewMat = _camera.GetViewMatrix();

    gl_using(_frame);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);

    // Upload deformed positions
    std::vector<glm::vec3> verts(_mesh.NumVertices());
    for (int i = 0; i < _mesh.NumVertices(); ++i) {
        verts[i] = eigen2glm(_mesh.positions[i]);
    }
    auto spanVerts = Engine::make_span_bytes<glm::vec3>(verts);

    // Compute per-vertex normals from surface faces
    std::vector<glm::vec3> normals(_mesh.NumVertices(), glm::vec3(0.0f));
    for (const auto & f : _mesh.surfaceFaces) {
        glm::vec3 p0 = verts[f[0]];
        glm::vec3 p1 = verts[f[1]];
        glm::vec3 p2 = verts[f[2]];
        glm::vec3 fn = glm::cross(p1 - p0, p2 - p0);
        normals[f[0]] += fn;
        normals[f[1]] += fn;
        normals[f[2]] += fn;
    }
    for (auto & n : normals) {
        float len = glm::length(n);
        if (len > 1e-10f) n /= len;
    }
    auto spanNormals = Engine::make_span_bytes<glm::vec3>(normals);

    // Draw surface
    if (_useLighting) {
        auto & prog = _litProgram;
        prog.GetUniforms().SetByName("u_Projection", projMat);
        prog.GetUniforms().SetByName("u_View", viewMat);
        prog.GetUniforms().SetByName("u_ViewPosition", _camera.Eye);
        prog.GetUniforms().SetByName("u_Color", _surfaceColor);
        prog.GetUniforms().SetByName("u_LightDir", glm::normalize(_lightDir));
        prog.GetUniforms().SetByName("u_LightColor", _lightIntensity * glm::vec3(1.0f));
        prog.GetUniforms().SetByName("u_AmbientColor", _ambientScale * _lightIntensity * glm::vec3(1.0f));
        prog.GetUniforms().SetByName("u_Shininess", _shininess);
        prog.GetUniforms().SetByName("u_FlatShading", int(_flatShading));

        _surfaceItem.UpdateVertexBuffer("position", spanVerts);
        _surfaceItem.UpdateVertexBuffer("normal", spanNormals);
        _surfaceItem.Draw({ prog.Use() });
    } else {
        _program.GetUniforms().SetByName("u_Projection", projMat);
        _program.GetUniforms().SetByName("u_View", viewMat);
        _program.GetUniforms().SetByName("u_Color", _surfaceColor);
        _surfaceItem.UpdateVertexBuffer("position", spanVerts);
        _surfaceItem.Draw({ _program.Use() });
    }

    // Draw wireframe (flat shader)
    if (_showWireframe) {
        glEnable(GL_LINE_SMOOTH);
        glLineWidth(0.5f);

        _program.GetUniforms().SetByName("u_Projection", projMat);
        _program.GetUniforms().SetByName("u_View", viewMat);
        _program.GetUniforms().SetByName("u_Color", glm::vec3(1.0f, 1.0f, 1.0f));
        _wireItem.UpdateVertexBuffer("position", spanVerts);
        _wireItem.Draw({ _program.Use() });

        glLineWidth(1.0f);
        glDisable(GL_LINE_SMOOTH);
    }

    glEnable(GL_CULL_FACE);

    return Common::CaseRenderResult {
        .Fixed     = false,
        .Flipped   = true,
        .Image     = _frame.GetColorAttachment(),
        .ImageSize = desiredSize,
    };
}

void CaseFEMSoftBody::OnProcessInput(ImVec2 const & pos) {
    _cameraManager.ProcessInput(_camera, pos);
    _forceManager.ProcessInput(_camera, pos);
}

} // namespace VCX::Labs::FEM
