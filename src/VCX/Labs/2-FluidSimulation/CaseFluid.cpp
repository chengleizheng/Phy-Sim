#include "Labs/2-FluidSimulation/CaseFluid.h"
#include "Labs/Common/ImGuiHelper.h"
#include "Engine/app.h"

static glm::vec3 eigen2glm(const Eigen::Vector3f & v) {
    return glm::vec3(v.x(), v.y(), v.z());
}

namespace VCX::Labs::Fluid {

    CaseFluid::CaseFluid() :
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/fluid.vert"),
                                        Engine::GL::SharedShader("assets/shaders/fluid.frag") })),
        _lineprogram(                                          // ← 补全
        Engine::GL::UniqueProgram({ 
            Engine::GL::SharedShader("assets/shaders/flat.vert"),
            Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _BoundaryItem(                                         // ← 补全
            Engine::GL::VertexLayout()
                .Add<glm::vec3>("position", Engine::GL::DrawFrequency::Static, 0),
            Engine::GL::PrimitiveType::Lines),
        _sphere(Engine::Sphere(4, _sim.grid.h * 0.17f), 0) {

        // ── 绑定 PassConstants ──
        _program.BindUniformBlock("PassConstants", 1);

        _lineprogram.GetUniforms().SetByName("u_Color", glm::vec3(1.0f));

        // ── 边框（静态，只上传一次）──
        float L = _sim.grid.h * _sim.grid.nx;
        _boundaryVerts = {
            {-0.025f, -0.025f, -0.025f}, {L+0.025f, -0.025f, -0.025f}, {L+0.025f, L+0.025f, -0.025f}, {-0.025f, L+0.025f, -0.025f},
            {-0.025f, -0.025f, L+0.025f}, {L+0.025f, -0.025f, L+0.025f}, {L+0.025f, L+0.025f, L+0.025f}, {-0.025f, L+0.025f, L+0.025f},
        };
        static const std::vector<std::uint32_t> lineIdx = {
            0,1, 1,2, 2,3, 3,0,
            4,5, 5,6, 6,7, 7,4,
            0,4, 1,5, 2,6, 3,7
        };
        _BoundaryItem.UpdateElementBuffer(lineIdx);            // ← 先传索引
        _BoundaryItem.UpdateVertexBuffer(                      // ← 再传顶点
            "position",
            Engine::make_span_bytes<glm::vec3>(_boundaryVerts));

        // ── 设置 shader 光照参数（固定默认值）──
        _program.GetUniforms().SetByName("u_AmbientScale",       1.0f);
        _program.GetUniforms().SetByName("u_UseBlinn",           1);
        _program.GetUniforms().SetByName("u_Shininess",          32.f);
        _program.GetUniforms().SetByName("u_UseGammaCorrection", 1);
        _program.GetUniforms().SetByName("u_AttenuationOrder",   2);

        // ── 设置默认场景光照 ──
        {
            Labs::Rendering::SceneObject::PassConstants pc {};
            pc.AmbientIntensity = glm::vec3(0.4f, 0.4f, 0.5f);
            pc.Lights[0] = { .Intensity = glm::vec3(1.2f, 1.2f, 1.2f), .Position = glm::vec3(0.f, 3.f, 0.f) };
            pc.Lights[1] = { .Intensity = glm::vec3(0.6f, 0.6f, 0.8f), .Position = glm::vec3(1.f, 1.f, 2.f) };
            pc.CntPointLights = 2;
            _sceneObject.PassConstantsBlock.Update(pc);
        }

        // ── 相机 ──
        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);

        // ── 初始化粒子 ──
        _sim.initializeParticles();
    }

    void CaseFluid::OnSetupPropsUI() {
        if (ImGui::CollapsingHeader("Simulation", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::SliderFloat("dt",           &_dt,                   0.001f, 0.05f);
            ImGui::SliderFloat("flipRatio",    &_sim.flipRatio,        0.f,    1.f,   "%.2f");
            ImGui::SliderInt  ("pressureIters",&_sim.numPressureIters, 1,      200);
            if (ImGui::Button("Reset")) {
                _sim.initializeParticles();
            }
            ImGui::SameLine();
        }
        ImGui::Spacing();
    }

    void CaseFluid::Advance() {
        _sim.StimulateTimestep(_numSubSteps, _dt);
    }

    Common::CaseRenderResult CaseFluid::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        Advance();

        _frame.Resize(desiredSize);
        _cameraManager.Update(_camera);

        // ── 更新 PassConstants（投影 / 视图）──
        _sceneObject.PassConstantsBlock.Update(
            &Labs::Rendering::SceneObject::PassConstants::Projection,
            _camera.GetProjectionMatrix(float(desiredSize.first) / float(desiredSize.second)));
        _sceneObject.PassConstantsBlock.Update(
            &Labs::Rendering::SceneObject::PassConstants::View,
            _camera.GetViewMatrix());
        _sceneObject.PassConstantsBlock.Update(
            &Labs::Rendering::SceneObject::PassConstants::ViewPosition,
            _camera.Eye);

        // ── 构建粒子 offset / color 数组 ──
        _particleOffsets.clear();
        _particleColors.clear();
        const int n = (int) _sim.particles.size();
        _particleOffsets.reserve(n);
        _particleColors.reserve(n);

        for (auto & p : _sim.particles) {
            _particleOffsets.push_back(eigen2glm(p.pos));
            float speed = p.vel.norm();
            float t     = std::min(speed / 3.0f, 1.0f);
            _particleColors.push_back(glm::vec3(0.1f + 0.1f * t, 0.35f + 0.25f * t, 0.55f + 0.45f * t));
        }

        // ── 绘制（教授建议的 ModelObject 方式）──
        gl_using(_frame);
        glEnable(GL_DEPTH_TEST);

        _lineprogram.GetUniforms().SetByName("u_Projection",
        _camera.GetProjectionMatrix(float(desiredSize.first)/float(desiredSize.second)));
        _lineprogram.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());
        _BoundaryItem.Draw({ _lineprogram.Use() });

        if (n > 0) {
            Rendering::ModelObject m = Rendering::ModelObject(_sphere, _particleOffsets, _particleColors);
            m.Mesh.Draw({ _program.Use() }, _sphere.Mesh.Indices.size(), 0, n);
        }

        glDisable(GL_DEPTH_TEST);

        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

    void CaseFluid::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);
    }

} // namespace VCX::Labs::Fluid
