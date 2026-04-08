#include "Labs/1-RigidBody/CaseTwoBodies.h"
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

    CaseTwoBodies::CaseTwoBodies():
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"),
                                        Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _boxItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
        _lineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines) {
        
        const std::vector<std::uint32_t> line_index = { 0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7 };
        _lineItem.UpdateElementBuffer(line_index);
        const std::vector<std::uint32_t> tri_index = { 0, 1, 2, 0, 2, 3, 1, 0, 4, 1, 4, 5, 1, 5, 6, 1, 6, 2, 2, 7, 3, 2, 6, 7, 0, 3, 7, 0, 7, 4, 4, 6, 5, 4, 7, 6 };
        _boxItem.UpdateElementBuffer(tri_index);
        
        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);

        // 初始化两个相向运动的刚体
        _box0.dim = Eigen::Vector3f(1.f, 1.f, 1.f);
        _box0.center = Eigen::Vector3f(-3.f, 0.f, 0.f);
        _box0.velocity = Eigen::Vector3f(2.f, 0.f, 0.f); // 向右
        _box0.mass = 1.0f;

        _box1.dim = Eigen::Vector3f(1.f, 1.f, 1.f);
        _box1.center = Eigen::Vector3f(3.f, 0.f, 0.f);
        _box1.velocity = Eigen::Vector3f(-2.f, 0.f, 0.f); // 向左
        _box1.mass = 1.0f;
    }

    void CaseTwoBodies::OnSetupPropsUI() {
        ImGui::Checkbox("Pause Simulation", &_pause);
        ImGui::SliderFloat("c", &_restitution, 0.0f, 1.0f);
        if (ImGui::CollapsingHeader("Appearance", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::ColorEdit3("Box0 Color", glm::value_ptr(_boxColor));
            ImGui::ColorEdit3("Box1 Color", glm::value_ptr(_boxColor));
        }
        if (ImGui::CollapsingHeader("Physics State", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::InputFloat("Box0 Mass", &_box0.mass);
            ImGui::InputFloat("Box1 Mass", &_box1.mass);
        }
        
        if (ImGui::Button("Reset Scenario: Face-Face Collision")) {
            _box0.center = Eigen::Vector3f(-3.f, 0.f, 0.f);
            _box0.velocity = Eigen::Vector3f(2.f, 0.f, 0.f);
            _box0.angularVelocity = Eigen::Vector3f(0.f, 0.f, 0.f);
            _box0.orientation = Eigen::Quaternionf(1.f, 0.f, 0.f, 0.f);

            _box1.center = Eigen::Vector3f(3.f, 0.f, 0.f);
            _box1.velocity = Eigen::Vector3f(-2.f, 0.f, 0.f);
            _box1.angularVelocity = Eigen::Vector3f(0.f, 0.f, 0.f);
            _box1.orientation = Eigen::Quaternionf(1.f, 0.f, 0.f, 0.f);
        }
        
        if (ImGui::Button("Reset Scenario: Edge-Edge Collision")) {


            _box0.center = Eigen::Vector3f(-3.f, 0.3f, 0.3f); // 偏移一点以产生旋转
            _box0.velocity = Eigen::Vector3f(2.f, 0.f, 0.f);
            _box0.angularVelocity = Eigen::Vector3f(0.f, 0.f, 0.f);
            _box0.orientation = Eigen::Quaternionf(Eigen::AngleAxisf(3.1415926f / 4.0f, Eigen::Vector3f::UnitY()));

            _box1.center = Eigen::Vector3f(3.f, 0.f, 0.f);
            _box1.velocity = Eigen::Vector3f(-2.f, 0.f, 0.f);
            _box1.angularVelocity = Eigen::Vector3f(0.f, 0.f, 0.f);
            _box1.orientation = Eigen::Quaternionf(Eigen::AngleAxisf(3.1415926f / 4.0f, Eigen::Vector3f::UnitZ()));
        }
        if (ImGui::Button("Reset Scenario: Vertex-Face Collision")) {
            _box0.center = Eigen::Vector3f(-3.f, 0.f, 0.f); // 更大的偏移产生擦边碰撞
            _box0.velocity = Eigen::Vector3f(2.f, 0.f, 0.f);
            _box0.angularVelocity = Eigen::Vector3f(0.f, 0.f, 0.f);
            _box0.orientation = Eigen::Quaternionf(1.f, 0.f, 0.f, 0.f);

            _box1.center = Eigen::Vector3f(3.f, 0.1f, 0.1f);
            _box1.velocity = Eigen::Vector3f(-2.f, 0.f, 0.f);
            _box1.angularVelocity = Eigen::Vector3f(0.f, 0.f, 0.f);
            _box1.orientation = Eigen::Quaternionf(0.9239f,0.2706f,0.2706f,0.2706f);
        }
    }

    void CaseTwoBodies::Advance(float timeDelta) {
        if (_pause) return;

        ProcessCollision();

        _box0.center += timeDelta * _box0.velocity;
        //  q(t+dt) = q(t) + Δq ⊗ q(t)
        Eigen::Quaternionf omega0(0,
            _box0.angularVelocity.x() * 0.5f * timeDelta,
            _box0.angularVelocity.y() * 0.5f * timeDelta,
            _box0.angularVelocity.z() * 0.5f * timeDelta);
        _box0.orientation.coeffs() += (omega0 * _box0.orientation).coeffs();
        _box0.orientation.normalize();

        _box1.center += timeDelta * _box1.velocity;
        Eigen::Quaternionf omega1(0,
            _box1.angularVelocity.x() * 0.5f * timeDelta,
            _box1.angularVelocity.y() * 0.5f * timeDelta,
            _box1.angularVelocity.z() * 0.5f * timeDelta);
        _box1.orientation.coeffs() += (omega1 * _box1.orientation).coeffs();
        _box1.orientation.normalize();
    }

    void CaseTwoBodies::ProcessCollision() {
        using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<float>>;
    
        CollisionGeometryPtr_t shape0(new fcl::Box<float>(_box0.dim.x(), _box0.dim.y(), _box0.dim.z()));
        CollisionGeometryPtr_t shape1(new fcl::Box<float>(_box1.dim.x(), _box1.dim.y(), _box1.dim.z()));

        fcl::Transform3f tf0(fcl::Translation3f(_box0.center) * _box0.orientation);
        fcl::Transform3f tf1(fcl::Translation3f(_box1.center) * _box1.orientation);

        fcl::CollisionObject<float> obj0(shape0, tf0);
        fcl::CollisionObject<float> obj1(shape1, tf1);

        fcl::CollisionRequest<float> request(8, true); 
        fcl::CollisionResult<float> result;

        fcl::collide(&obj0, &obj1, request, result);

        if (result.isCollision()) {
            std::vector<fcl::Contact<float>> contacts;
            result.getContacts(contacts);
        
            if (contacts.empty()) return;

            Eigen::Vector3f avg_pos = Eigen::Vector3f::Zero();
            Eigen::Vector3f avg_normal = Eigen::Vector3f::Zero();
            float max_depth = 0.0f;

            for (const auto& contact : contacts) {
                avg_pos += contact.pos;
            
                // 统一法线方向：强制让法线 n 始终从 Box 1 指向 Box 0
                Eigen::Vector3f n = contact.normal;
                if (n.dot(_box0.center - _box1.center) < 0) {
                    n = -n;
                }
                avg_normal += n;
                max_depth = std::max(max_depth, contact.penetration_depth);
            }

            avg_pos /= contacts.size();
            avg_normal.normalize();

            // 对这个等效的中心点施加一次冲量
            ApplyImpulse(_box0, _box1, avg_pos, avg_normal, max_depth);
        }
    }

    void CaseTwoBodies::ApplyImpulse(Box& boxA, Box& boxB, const Eigen::Vector3f& p, const Eigen::Vector3f& n, float depth) {
        Eigen::Vector3f rA = p - boxA.center;
        Eigen::Vector3f rB = p - boxB.center;

        Eigen::Vector3f vA_p = boxA.velocity + boxA.angularVelocity.cross(rA);
        Eigen::Vector3f vB_p = boxB.velocity + boxB.angularVelocity.cross(rB);

        Eigen::Vector3f v_rel = vA_p - vB_p;

        float relVelAlongNormal = v_rel.dot(n);
        if (relVelAlongNormal > 0) {
            return; 
        }

        Eigen::Matrix3f invIa = boxA.GetInertiaMatrix().inverse();
        Eigen::Matrix3f invIb = boxB.GetInertiaMatrix().inverse();

        // 课件 P33 的刚体冲量分母计算 [cite: 2]
        float termA = n.dot( (invIa * (rA.cross(n))).cross(rA) );
        float termB = n.dot( (invIb * (rB.cross(n))).cross(rB) );
    
        float denominator = (1.0f / boxA.mass) + (1.0f / boxB.mass) + termA + termB;
    
        // 计算冲量标量 j (一定是正数) [cite: 2]
        float j = -(1.0f + _restitution) * relVelAlongNormal / denominator;

        Eigen::Vector3f J = j * n;

        // 更新线速度与角速度 [cite: 2]
        boxA.velocity += J / boxA.mass;
        boxA.angularVelocity += invIa * rA.cross(J);

        boxB.velocity -= J / boxB.mass;
        boxB.angularVelocity -= invIb * rB.cross(J);

        // 位置修正 (Position Correction)
        const float percent = 0.8f; 
        const float slop = 0.01f;
        Eigen::Vector3f correction = (std::max(depth - slop, 0.0f) / ((1.0f / boxA.mass) + (1.0f / boxB.mass))) * percent * n;
    
        boxA.center += correction * (1.0f / boxA.mass);
        boxB.center -= correction * (1.0f / boxB.mass);
    }

    void CaseTwoBodies::GetBoxVertices(const Box& box, std::vector<glm::vec3>& outVertices) {
        glm::vec3 center = eigen2glm(box.center);
        Eigen::Matrix3f R = box.orientation.toRotationMatrix();

        glm::vec3 new_x = eigen2glm(R * Eigen::Vector3f(box.dim.x() / 2, 0.f, 0.f));
        glm::vec3 new_y = eigen2glm(R * Eigen::Vector3f(0.f, box.dim.y() / 2, 0.f));
        glm::vec3 new_z = eigen2glm(R * Eigen::Vector3f(0.f, 0.f, box.dim.z() / 2));

        outVertices.resize(8);
        outVertices[0] = center - new_x + new_y + new_z;
        outVertices[1] = center + new_x + new_y + new_z;
        outVertices[2] = center + new_x + new_y - new_z;
        outVertices[3] = center - new_x + new_y - new_z;
        outVertices[4] = center - new_x - new_y + new_z;
        outVertices[5] = center + new_x - new_y + new_z;
        outVertices[6] = center + new_x - new_y - new_z;
        outVertices[7] = center - new_x - new_y - new_z;
    }

    Common::CaseRenderResult CaseTwoBodies::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        Advance(Engine::GetDeltaTime());

        _frame.Resize(desiredSize);
        _cameraManager.Update(_camera);
        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());

        gl_using(_frame);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LINE_SMOOTH);

        // 渲染 Box 0
        std::vector<glm::vec3> verts0;
        GetBoxVertices(_box0, verts0);
        auto span_bytes0 = Engine::make_span_bytes<glm::vec3>(verts0);

        _program.GetUniforms().SetByName("u_Color", _boxColor);
        _boxItem.UpdateVertexBuffer("position", span_bytes0);
        _boxItem.Draw({ _program.Use() });
        
        _program.GetUniforms().SetByName("u_Color", glm::vec3(1.f, 1.f, 1.f));
        _lineItem.UpdateVertexBuffer("position", span_bytes0);
        _lineItem.Draw({ _program.Use() });

        // 渲染 Box 1
        std::vector<glm::vec3> verts1;
        GetBoxVertices(_box1, verts1);
        auto span_bytes1 = Engine::make_span_bytes<glm::vec3>(verts1);
        
        _program.GetUniforms().SetByName("u_Color", _boxColor); 
        _boxItem.UpdateVertexBuffer("position", span_bytes1);
        _boxItem.Draw({ _program.Use() });
        
        _program.GetUniforms().SetByName("u_Color", glm::vec3(1.f, 1.f, 1.f));
        _lineItem.UpdateVertexBuffer("position", span_bytes1);
        _lineItem.Draw({ _program.Use() });

        glDisable(GL_LINE_SMOOTH);

        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

    void CaseTwoBodies::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);
        // 鼠标施力逻辑可根据需要开启
        // _forceManager.ProcessInput(_camera, pos); 
    }

} // namespace VCX::Labs::RigidBody