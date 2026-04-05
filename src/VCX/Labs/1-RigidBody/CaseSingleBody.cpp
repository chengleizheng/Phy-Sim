#include "Labs/1-RigidBody/CaseSingleBody.h"
#include "Labs/Common/ImGuiHelper.h"
#include "Engine/app.h"
#include <iostream>

static glm::vec3 eigen2glm(const Eigen::Vector3f& eigenVec) {
    return glm::vec3(eigenVec.x(), eigenVec.y(), eigenVec.z());
}

static Eigen::Vector3f glm2eigen(const glm::vec3& glmVec) {
    return Eigen::Vector3f(glmVec.x, glmVec.y, glmVec.z);
}

namespace VCX::Labs::RigidBody {

    CaseSingleBody::CaseSingleBody():
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"),
                                        Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _boxItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
        _lineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines) {
        //     3-----2
        //    /|    /|
        //   0 --- 1 |
        //   | 7 - | 6
        //   |/    |/
        //   4 --- 5
        const std::vector<std::uint32_t> line_index = { 0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7 }; // line index
        _lineItem.UpdateElementBuffer(line_index);

        // const std::vector<std::uint32_t> tri_index = { 0, 1, 2, 0, 2, 3, 1, 4, 0, 1, 4, 5, 1, 6, 5, 1, 2, 6, 2, 3, 7, 2, 6, 7, 0, 3, 7, 0, 4, 7, 4, 5, 6, 4, 6, 7 };
        const std::vector<std::uint32_t> tri_index = { 0, 1, 2, 0, 2, 3, 1, 0, 4, 1, 4, 5, 1, 5, 6, 1, 6, 2, 2, 7, 3, 2, 6, 7, 0, 3, 7, 0, 7, 4, 4, 6, 5, 4, 7, 6 };
        _boxItem.UpdateElementBuffer(tri_index);
        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);
    }

    void CaseSingleBody::OnSetupPropsUI() {
    if (ImGui::CollapsingHeader("Appearance", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::ColorEdit3("Box Color", glm::value_ptr(_boxColor));
        ImGui::SliderFloat("x", &_dim[0], 0.5, 4);
        ImGui::SliderFloat("y", &_dim[1], 0.5, 4);
        ImGui::SliderFloat("z", &_dim[2], 0.5, 4);
    }
    
    if (ImGui::CollapsingHeader("Physics State", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::InputFloat("Mass", &_box.mass);
        ImGui::DragFloat3("Velocity", _box.velocity.data(), 0.01f);
        ImGui::DragFloat3("Angular Vel", _box.angularVelocity.data(), 0.01f);
        
        if (ImGui::Button("Reset State")) {
            _box.center = Eigen::Vector3f(0.f, 0.f, 0.f);
            _box.velocity = Eigen::Vector3f(0.f, 0.f, 0.f);
            _box.angularVelocity = Eigen::Vector3f(0.f, 1.0f, 0.f); // 恢复自转
            _box.orientation = Eigen::Quaternionf(1.f, 0.f, 0.f, 0.f);
        }
    }
    ImGui::Spacing();
    }

    Common::CaseRenderResult CaseSingleBody::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        // apply mouse control first
        std::pair<glm::vec3,glm::vec3> force =  _forceManager.getForce(eigen2glm(_box.center));
        OnProcessMouseControl(force);

        Advance(Engine::GetDeltaTime());

        // rendering
        _frame.Resize(desiredSize);

        _cameraManager.Update(_camera);
        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());

        gl_using(_frame);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LINE_SMOOTH);
        glLineWidth(.5f);

        std::vector<glm::vec3> VertsPosition;
        _box.dim = glm2eigen(_dim); // 确保 UI 调节的尺寸能同步给物理 Box 用于计算惯性张量
        
        // 提取物理引擎更新后的中心点
        glm::vec3 current_center = eigen2glm(_box.center);
        
        // 将四元数转换为旋转矩阵
        Eigen::Matrix3f R = _box.orientation.toRotationMatrix();

        // 将局部坐标轴的基向量乘上旋转矩阵，转换到世界坐标系
        glm::vec3 new_x = eigen2glm(R * Eigen::Vector3f(_box.dim.x() / 2, 0.f, 0.f));
        glm::vec3 new_y = eigen2glm(R * Eigen::Vector3f(0.f, _box.dim.y() / 2, 0.f));
        glm::vec3 new_z = eigen2glm(R * Eigen::Vector3f(0.f, 0.f, _box.dim.z() / 2));

        VertsPosition.resize(8);
        // 基于实时 center 和旋转后的基向量构建顶点
        VertsPosition[0] = current_center - new_x + new_y + new_z;
        VertsPosition[1] = current_center + new_x + new_y + new_z;
        VertsPosition[2] = current_center + new_x + new_y - new_z;
        VertsPosition[3] = current_center - new_x + new_y - new_z;
        VertsPosition[4] = current_center - new_x - new_y + new_z;
        VertsPosition[5] = current_center + new_x - new_y + new_z;
        VertsPosition[6] = current_center + new_x - new_y - new_z;
        VertsPosition[7] = current_center - new_x - new_y - new_z;

        auto span_bytes = Engine::make_span_bytes<glm::vec3>(VertsPosition);

        _program.GetUniforms().SetByName("u_Color", _boxColor);
        _boxItem.UpdateVertexBuffer("position", span_bytes);
        _boxItem.Draw({ _program.Use() });

        _program.GetUniforms().SetByName("u_Color", glm::vec3(1.f, 1.f, 1.f));
        _lineItem.UpdateVertexBuffer("position", span_bytes);
        _lineItem.Draw({ _program.Use() });

        glLineWidth(1.f);
        glPointSize(1.f);
        glDisable(GL_LINE_SMOOTH);

        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

    void CaseSingleBody::Advance(float timeDelta) {
        _box.center += timeDelta * _box.velocity;   // update position

        Eigen::Quaternionf _angularVelocityQuaternion(0, _box.angularVelocity.x() * timeDelta * 0.5f, _box.angularVelocity.y() * timeDelta * 0.5f, _box.angularVelocity.z() * timeDelta * 0.5f);

        _box.orientation.coeffs() += (_angularVelocityQuaternion*_box.orientation).coeffs();
        _box.orientation.normalize();
    }

    void CaseSingleBody::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);
        _forceManager.ProcessInput(_camera, pos);
    }

    void CaseSingleBody::OnProcessMouseControl(std::pair<glm::vec3, glm::vec3> force) {
    glm::vec3 forceDelta = force.first;
    glm::vec3 forcePoint = force.second;

    // 如果鼠标没有拖拽（没有产生力），直接返回，避免无意义的计算
    if (glm::length(forceDelta) < 1e-6f) return;

    // 将力视为冲量 (Impulse)
    Eigen::Vector3f impulse = glm2eigen(forceDelta);
    Eigen::Vector3f r = glm2eigen(forcePoint) - _box.center; // 力臂
    
    // 角冲量 = 力臂 x 冲量
    Eigen::Vector3f angularImpulse = r.cross(impulse);

    // 线速度更新：v = v + J / m
    _box.velocity += impulse / _box.mass;
    
    // 角速度更新：ω = ω + I^-1 * (r x J)
    _box.angularVelocity += (_box.GetInertiaMatrix().inverse()) * angularImpulse;
}

} // namespace VCX::Labs::RigidBody
