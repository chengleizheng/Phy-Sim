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

            ImGui::InputFloat("pos_x", &_center[0]);
            ImGui::InputFloat("pos_y", &_center[1]);
            ImGui::InputFloat("pos_z", &_center[2]);
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
        glm::vec3              new_x = _dim[0] / 2 * glm::vec3(1.f, 0.f, 0.f);
        glm::vec3              new_y = _dim[1] / 2 * glm::vec3(0.f, 1.f, 0.f);
        glm::vec3              new_z = _dim[2] / 2 * glm::vec3(0.f, 0.f, 1.f);
        VertsPosition.resize(8);
        VertsPosition[0] = _center - new_x + new_y + new_z;
        VertsPosition[1] = _center + new_x + new_y + new_z;
        VertsPosition[2] = _center + new_x + new_y - new_z;
        VertsPosition[3] = _center - new_x + new_y - new_z;
        VertsPosition[4] = _center - new_x - new_y + new_z;
        VertsPosition[5] = _center + new_x - new_y + new_z;
        VertsPosition[6] = _center + new_x - new_y - new_z;
        VertsPosition[7] = _center - new_x - new_y - new_z;

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
        float movingScale = 1.f;

        Eigen::Vector3f torque = (glm2eigen(forcePoint) - _box.center).cross(glm2eigen(forceDelta));
        _box.angularVelocity += (_box.GetInertiaMatrix().inverse())*torque;
        _box.velocity += glm2eigen(forceDelta) * movingScale;
    }

} // namespace VCX::Labs::RigidBody
