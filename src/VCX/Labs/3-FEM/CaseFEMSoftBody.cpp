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
    _surfaceItem(
        Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0),
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
    ImGui::TextColored(ImVec4(0.5f, 1.0f, 0.5f, 1.0f),
            ImGui::IsKeyDown(ImGuiKey_Space) ? "[SPACE] held - lifting!" : "Hold [SPACE] to lift");
    if (ImGui::CollapsingHeader("Appearance", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::ColorEdit3("Surface Color", glm::value_ptr(_surfaceColor));
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
        if (ImGui::Button("Reset")) {
            ResetSimulation();
        }
    }
    ImGui::Spacing();
}

void CaseFEMSoftBody::Advance(float dt) {
    if (dt <= 0.0f) return;

    // 将帧时间钳制在合理范围（避免失焦等极端情况）
    dt = std::min(dt, 1.0f / 30.0f);

    // 刚度越大（lambda/mu 越大）需要越多子步
    const float subDt = dt / float(_numSubsteps);
    const Eigen::Vector3f grav3 = glm2eigen(_gravity);
    const bool spaceHeld = ImGui::IsKeyDown(ImGuiKey_Space);

    for (int step = 0; step < _numSubsteps; ++step) {
        // 1. 计算弹性力
        std::vector<Eigen::Vector3f> forces;
        _integrator.ComputeAllForces(_mesh, forces);

        // 2. 叠加外力，更新速度和位置
        for (int i = 0; i < _mesh.NumVertices(); ++i) {
            if (_mesh.fixed[i]) {
                _mesh.positions[i]  = _mesh.restPositions[i];
                _mesh.velocities[i] = Eigen::Vector3f::Zero();
                continue;
            }

            // 重力 + 空格键抬升
            forces[i] += _mesh.masses[i] * grav3;
            if (spaceHeld) {
                forces[i] += Eigen::Vector3f(0.0f, _liftForce * _mesh.masses[i], 0.0f);
            }

            // 速度更新（显式欧拉）
            _mesh.velocities[i] += subDt * forces[i] / _mesh.masses[i];

            // 用指数衰减施加阻尼，无条件稳定
            const float dampFactor = std::exp(-_damping * subDt);
            _mesh.velocities[i] *= dampFactor;

            // 位置更新
            _mesh.positions[i] += subDt * _mesh.velocities[i];

            // 地板碰撞
            if (_mesh.positions[i].y() < _floorY) {
                _mesh.positions[i].y() = _floorY;
                if (_mesh.velocities[i].y() < 0.0f) {
                    _mesh.velocities[i].y() = 0.0f;
                }
            }
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
    _program.GetUniforms().SetByName("u_Projection",
        _camera.GetProjectionMatrix(float(desiredSize.first) / float(desiredSize.second)));
    _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());

    gl_using(_frame);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);

    // Upload current deformed positions
    std::vector<glm::vec3> verts(_mesh.NumVertices());
    for (int i = 0; i < _mesh.NumVertices(); ++i) {
        verts[i] = eigen2glm(_mesh.positions[i]);
    }
    auto spanVerts = Engine::make_span_bytes<glm::vec3>(verts);

    // Draw filled surface triangles
    _program.GetUniforms().SetByName("u_Color", _surfaceColor);
    _surfaceItem.UpdateVertexBuffer("position", spanVerts);
    _surfaceItem.Draw({ _program.Use() });

    // Draw wireframe
    if (_showWireframe) {
        glEnable(GL_LINE_SMOOTH);
        glLineWidth(0.5f);

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
