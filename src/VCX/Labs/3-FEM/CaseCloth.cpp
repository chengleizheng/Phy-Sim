#include "CaseCloth.h"
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

// --- plane-stress Lamé conversion ---
static void E_nu_to_lame(float E, float nu, float & lambda, float & mu) {
    mu     = E / (2.0f * (1.0f + nu));
    lambda = E * nu / (1.0f - nu * nu);
}

CaseCloth::CaseCloth() :
    _program(
        Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"),
                                    Engine::GL::SharedShader("assets/shaders/flat.frag") })),
    _litProgram(
        Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/lit_mesh.vert"),
                                    Engine::GL::SharedShader("assets/shaders/lit_mesh.frag") })),
    _clothItem(
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
    UpdateMaterialFromE();
    RebuildMesh();
}

void CaseCloth::UpdateMaterialFromE() {
    E_nu_to_lame(_E, _nu, _integrator.material.lambda, _integrator.material.mu);
}

void CaseCloth::RebuildMesh() {
    _mesh.BuildGrid(_gridResX, _gridResY,
                    _clothWidth, _clothHeight,
                    _totalMass, _initHeight);

    if (_pinCorners) {
        _mesh.FixCorner(_clothWidth * 0.01f, _clothHeight * 0.99f);
    }

    // Triangle index buffer
    {
        std::vector<std::uint32_t> triIdx;
        triIdx.reserve(_mesh.NumTriangles() * 3);
        for (const auto & f : _mesh.triangles) {
            triIdx.push_back(f[0]);
            triIdx.push_back(f[1]);
            triIdx.push_back(f[2]);
        }
        _clothItem.UpdateElementBuffer(triIdx);
    }

    // Wireframe line index buffer (deduplicated edges)
    {
        std::set<std::pair<int, int>> edgeSet;
        for (const auto & f : _mesh.triangles) {
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

void CaseCloth::ResetSimulation() {
    for (int i = 0; i < _mesh.NumVertices(); ++i) {
        _mesh.positions[i]  = _mesh.restPositions[i];
        _mesh.velocities[i] = Eigen::Vector3f::Zero();
    }
}

void CaseCloth::OnSetupPropsUI() {
    ImGui::TextColored(ImVec4(0.5f, 1.0f, 0.5f, 1.0f),
        ImGui::IsKeyDown(ImGuiKey_Space) ? "[SPACE] held - blowing wind!" : "Hold [SPACE] to blow wind");

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
        bool matChanged = false;
        if (matChanged) UpdateMaterialFromE();

        ImGui::SliderFloat("Mass", &_totalMass, 0.01f, 1.0f);
        ImGui::SliderFloat("Damping", &_damping, 0.0f, 10.0f);
        ImGui::SliderFloat3("Gravity", glm::value_ptr(_gravity), -20.0f, 0.0f);
    }

    if (ImGui::CollapsingHeader("Mesh", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SliderInt("Grid X", &_gridResX, 5, 40);
        ImGui::SliderInt("Grid Y", &_gridResY, 5, 40);
        ImGui::SliderFloat("Width", &_clothWidth, 0.2f, 2.0f);
        ImGui::SliderFloat("Height", &_clothHeight, 0.2f, 2.0f);
        ImGui::SliderFloat("Init Height", &_initHeight, 0.5f, 5.0f);
        ImGui::Checkbox("Pin Corners", &_pinCorners);
        if (ImGui::Button("Rebuild Mesh")) {
            _needsRebuild = true;
        }
    }

    if (ImGui::CollapsingHeader("Interaction", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SliderFloat("Wind Force", &_windForce, 0.0f, 100.0f);
    }

    if (ImGui::CollapsingHeader("Simulation", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SliderInt("Substeps", &_numSubsteps, 5, 300);
        if (ImGui::Button("Reset")) {
            ResetSimulation();
        }
    }
    ImGui::Spacing();
}

void CaseCloth::Advance(float dt) {
    if (dt <= 0.0f) return;

    dt = std::min(dt, 1.0f / 30.0f);
    const float subDt = dt / float(_numSubsteps);
    const Eigen::Vector3f grav3 = glm2eigen(_gravity);
    const bool spaceHeld = ImGui::IsKeyDown(ImGuiKey_Space);

    for (int step = 0; step < _numSubsteps; ++step) {
        std::vector<Eigen::Vector3f> forces;
        _integrator.ComputeAllForces(_mesh, forces);

        for (int i = 0; i < _mesh.NumVertices(); ++i) {
            if (_mesh.pinned[i]) {
                _mesh.positions[i]  = _mesh.restPositions[i];
                _mesh.velocities[i] = Eigen::Vector3f::Zero();
                continue;
            }

            // NaN guard: if position or force went bad, reset vertex to rest
            if (!_mesh.positions[i].allFinite() || !forces[i].allFinite()) {
                _mesh.positions[i]  = _mesh.restPositions[i];
                _mesh.velocities[i] = Eigen::Vector3f::Zero();
                continue;
            }

            // Gravity
            forces[i] += _mesh.masses[i] * grav3;

            // Space = wind (blows in +Z direction, slightly up)
            if (spaceHeld) {
                forces[i] += Eigen::Vector3f(0.0f, _windForce * 0.3f * _mesh.masses[i],
                                                   _windForce * _mesh.masses[i]);
            }

            // Velocity clamp: prevent explosion
            _mesh.velocities[i] += subDt * forces[i] / _mesh.masses[i];
            float speed = _mesh.velocities[i].norm();
            if (speed > 100.0f) {
                _mesh.velocities[i] *= 100.0f / speed;
            }

            const float dampFactor = std::exp(-_damping * subDt);
            _mesh.velocities[i] *= dampFactor;

            _mesh.positions[i] += subDt * _mesh.velocities[i];

            // Floor collision
            if (_mesh.positions[i].y() < _floorY) {
                _mesh.positions[i].y() = _floorY;
                if (_mesh.velocities[i].y() < 0.0f) {
                    _mesh.velocities[i].y() = 0.0f;
                }
            }
        }
    }
}

void CaseCloth::ApplyMouseForce() {
    std::vector<glm::vec3> candidates;
    candidates.reserve(_mesh.NumVertices());
    for (int i = 0; i < _mesh.NumVertices(); ++i) {
        candidates.push_back(eigen2glm(_mesh.positions[i]));
    }

    auto [force, idx] = _forceManager.getForce(candidates);

    if (glm::length(force) > 1e-6f && idx >= 0 && !_mesh.pinned[idx]) {
        _mesh.velocities[idx] += glm2eigen(force) / _mesh.masses[idx];
    }
}

Common::CaseRenderResult CaseCloth::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
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

    // Compute per-vertex normals from triangles
    std::vector<glm::vec3> normals(_mesh.NumVertices(), glm::vec3(0.0f));
    for (const auto & f : _mesh.triangles) {
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

        _clothItem.UpdateVertexBuffer("position", spanVerts);
        _clothItem.UpdateVertexBuffer("normal", spanNormals);
        _clothItem.Draw({ prog.Use() });
    } else {
        _program.GetUniforms().SetByName("u_Projection", projMat);
        _program.GetUniforms().SetByName("u_View", viewMat);
        _program.GetUniforms().SetByName("u_Color", _surfaceColor);
        _clothItem.UpdateVertexBuffer("position", spanVerts);
        _clothItem.Draw({ _program.Use() });
    }

    // Wireframe
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

void CaseCloth::OnProcessInput(ImVec2 const & pos) {
    _cameraManager.ProcessInput(_camera, pos);
    _forceManager.ProcessInput(_camera, pos);
}

} // namespace VCX::Labs::FEM
