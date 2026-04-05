#include "Labs/1-RigidBody/CaseComplexScene.h"
#include "Labs/Common/ImGuiHelper.h"
#include "Engine/app.h"

static glm::vec3 eigen2glm(const Eigen::Vector3f& eigenVec) { return glm::vec3(eigenVec.x(), eigenVec.y(), eigenVec.z()); }
static Eigen::Vector3f glm2eigen(const glm::vec3& glmVec) { return Eigen::Vector3f(glmVec.x, glmVec.y, glmVec.z); }

namespace VCX::Labs::RigidBody {

    CaseComplexScene::CaseComplexScene():
        _program(Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"), Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        _boxItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
        _lineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines) {
        
        const std::vector<std::uint32_t> line_index = { 0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7 };
        _lineItem.UpdateElementBuffer(line_index);
        const std::vector<std::uint32_t> tri_index = { 0, 1, 2, 0, 2, 3, 1, 0, 4, 1, 4, 5, 1, 5, 6, 1, 6, 2, 2, 7, 3, 2, 6, 7, 0, 3, 7, 0, 7, 4, 4, 6, 5, 4, 7, 6 };
        _boxItem.UpdateElementBuffer(tri_index);
        
        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);

        ResetScene();
    }

    void CaseComplexScene::ResetScene() {
        _dynamicBoxes.clear();
        _staticBoxes.clear();

        // 1. 构建静态环境 (mass = 0 表示静态)
        // 地板
        Box floor(Eigen::Vector3f(15.f, 0.5f, 15.f), Eigen::Vector3f(0.f, -0.25f, 0.f), Eigen::Quaternionf::Identity(), 0.f);
        floor.boxColor = glm::vec3(0.5f, 0.5f, 0.5f);
        _staticBoxes.push_back(floor);

        // 两堵墙
        Box wall1(Eigen::Vector3f(0.5f, 5.f, 15.f), Eigen::Vector3f(-7.5f, 2.5f, 0.f), Eigen::Quaternionf::Identity(), 0.f);
        wall1.boxColor = glm::vec3(0.6f, 0.6f, 0.6f);
        _staticBoxes.push_back(wall1);
        
        Box wall2(Eigen::Vector3f(0.5f, 5.f, 15.f), Eigen::Vector3f(7.5f, 2.5f, 0.f), Eigen::Quaternionf::Identity(), 0.f);
        wall2.boxColor = glm::vec3(0.6f, 0.6f, 0.6f);
        _staticBoxes.push_back(wall2);

        // 2. 构建动态方块 (产生复杂碰撞)
        for (int i = 0; i < 5; ++i) {
            Box b;
            b.mass = 1.0f;
            b.dim = Eigen::Vector3f(1.f, 1.f, 1.f);
            // 错开高度和一点点水平位置，使其产生连环撞击
            b.center = Eigen::Vector3f(0.1f * i, 3.0f + i * 2.0f, 0.1f * i);
            b.velocity = Eigen::Vector3f::Zero();
            // 给定微小初始旋转，打破对称性
            b.orientation = Eigen::Quaternionf(Eigen::AngleAxisf(0.2f * i, Eigen::Vector3f::UnitY()));
            b.boxColor = glm::vec3(0.2f + 0.15f * i, 0.8f - 0.1f * i, 0.3f + 0.1f * i);
            _dynamicBoxes.push_back(b);
        }
    }

    void CaseComplexScene::OnSetupPropsUI() {
        ImGui::Checkbox("Pause Simulation", &_pause);
        ImGui::SliderFloat("Restitution", &_restitution, 0.0f, 1.0f);
        if (ImGui::Button("Reset Scene")) {
            ResetScene();
        }
    }

    void CaseComplexScene::Advance(float timeDelta) {
        if (_pause) return;

        // 1. 积分与外力应用
        for (auto& box : _dynamicBoxes) {
            // 施加重力
            box.velocity += glm2eigen(_gravity) * timeDelta;
            
            // 施加阻尼 (缓解抖动)
            box.velocity *= _linearDamping;
            box.angularVelocity *= _angularDamping;

            // 更新位置与旋转
            box.center += timeDelta * box.velocity;
            Eigen::Quaternionf dq(0, box.angularVelocity.x() * timeDelta * 0.5f, box.angularVelocity.y() * timeDelta * 0.5f, box.angularVelocity.z() * timeDelta * 0.5f);
            box.orientation.coeffs() += (dq * box.orientation).coeffs();
            box.orientation.normalize();
        }

        // 2. 碰撞检测与处理
        ProcessCollisions();
    }

    void CaseComplexScene::ProcessCollisions() {
        using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<float>>;
        fcl::CollisionRequest<float> request(8, true);

        // 准备动态盒子的 FCL 对象
        std::vector<fcl::CollisionObject<float>> dynObjs;
        for (const auto& box : _dynamicBoxes) {
            CollisionGeometryPtr_t shape(new fcl::Box<float>(box.dim.x(), box.dim.y(), box.dim.z()));
            fcl::Transform3f tf(fcl::Translation3f(box.center) * box.orientation);
            dynObjs.emplace_back(shape, tf);
        }

        // 准备静态盒子的 FCL 对象
        std::vector<fcl::CollisionObject<float>> statObjs;
        for (const auto& box : _staticBoxes) {
            CollisionGeometryPtr_t shape(new fcl::Box<float>(box.dim.x(), box.dim.y(), box.dim.z()));
            fcl::Transform3f tf(fcl::Translation3f(box.center) * box.orientation);
            statObjs.emplace_back(shape, tf);
        }

        // A. 动态 vs 静态
        for (size_t i = 0; i < _dynamicBoxes.size(); ++i) {
            for (size_t j = 0; j < _staticBoxes.size(); ++j) {
                fcl::CollisionResult<float> result;
                fcl::collide(&dynObjs[i], &statObjs[j], request, result);
                if (result.isCollision()) {
                    std::vector<fcl::Contact<float>> contacts;
                    result.getContacts(contacts);
                    
                    Eigen::Vector3f avg_pos = Eigen::Vector3f::Zero();
                    Eigen::Vector3f avg_normal = Eigen::Vector3f::Zero();
                    float max_depth = 0.0f;

                    for (const auto& c : contacts) {
                        avg_pos += c.pos;
                        Eigen::Vector3f n = c.normal;
                        // 确保法线从 Static(B) 指向 Dynamic(A)
                        if (n.dot(_dynamicBoxes[i].center - _staticBoxes[j].center) < 0) n = -n;
                        avg_normal += n;
                        max_depth = std::max(max_depth, c.penetration_depth);
                    }
                    avg_pos /= contacts.size();
                    avg_normal.normalize();
                    ApplyImpulse(_dynamicBoxes[i], _staticBoxes[j], avg_pos, avg_normal, max_depth);
                }
            }
        }

        // B. 动态 vs 动态
        for (size_t i = 0; i < _dynamicBoxes.size(); ++i) {
            for (size_t j = i + 1; j < _dynamicBoxes.size(); ++j) {
                fcl::CollisionResult<float> result;
                fcl::collide(&dynObjs[i], &dynObjs[j], request, result);
                if (result.isCollision()) {
                    std::vector<fcl::Contact<float>> contacts;
                    result.getContacts(contacts);
                    
                    Eigen::Vector3f avg_pos = Eigen::Vector3f::Zero();
                    Eigen::Vector3f avg_normal = Eigen::Vector3f::Zero();
                    float max_depth = 0.0f;

                    for (const auto& c : contacts) {
                        avg_pos += c.pos;
                        Eigen::Vector3f n = c.normal;
                        if (n.dot(_dynamicBoxes[i].center - _dynamicBoxes[j].center) < 0) n = -n;
                        avg_normal += n;
                        max_depth = std::max(max_depth, c.penetration_depth);
                    }
                    avg_pos /= contacts.size();
                    avg_normal.normalize();
                    ApplyImpulse(_dynamicBoxes[i], _dynamicBoxes[j], avg_pos, avg_normal, max_depth);
                }
            }
        }
    }

    void CaseComplexScene::ApplyImpulse(Box& boxA, Box& boxB, const Eigen::Vector3f& p, const Eigen::Vector3f& n, float depth) {
        Eigen::Vector3f rA = p - boxA.center;
        Eigen::Vector3f rB = p - boxB.center;

        Eigen::Vector3f vA_p = boxA.velocity + boxA.angularVelocity.cross(rA);
        Eigen::Vector3f vB_p = boxB.velocity + boxB.angularVelocity.cross(rB);
        Eigen::Vector3f v_rel = vA_p - vB_p;

        float relVelAlongNormal = v_rel.dot(n);
        if (relVelAlongNormal > 0) return;

        // 处理质量与惯性张量 (兼容静态刚体)
        float invMassA = boxA.mass > 0.0f ? 1.0f / boxA.mass : 0.0f;
        float invMassB = boxB.mass > 0.0f ? 1.0f / boxB.mass : 0.0f;
        Eigen::Matrix3f invIa = Eigen::Matrix3f::Zero();
if (boxA.mass > 0.0f) {
    invIa = boxA.GetInertiaMatrix().inverse();
}

Eigen::Matrix3f invIb = Eigen::Matrix3f::Zero();
if (boxB.mass > 0.0f) {
    invIb = boxB.GetInertiaMatrix().inverse();
}
        float termA = n.dot( (invIa * (rA.cross(n))).cross(rA) );
        float termB = n.dot( (invIb * (rB.cross(n))).cross(rB) );
        
        float denominator = invMassA + invMassB + termA + termB;
        if (denominator < 1e-6f) return; // 避免除零

        float j = -(1.0f + _restitution) * relVelAlongNormal / denominator;
        Eigen::Vector3f J = j * n;

        // 仅对动态刚体施加冲量
        if (invMassA > 0.f) {
            boxA.velocity += J * invMassA;
            boxA.angularVelocity += invIa * rA.cross(J);
        }
        if (invMassB > 0.f) {
            boxB.velocity -= J * invMassB;
            boxB.angularVelocity -= invIb * rB.cross(J);
        }

        // 位置补偿 (防止沉入地面)
        const float percent = 0.5f; 
        const float slop = 0.01f;
        Eigen::Vector3f correction = (std::max(depth - slop, 0.0f) / (invMassA + invMassB)) * percent * n;
        
        if (invMassA > 0.f) boxA.center += correction * invMassA;
        if (invMassB > 0.f) boxB.center -= correction * invMassB;
    }

    void CaseComplexScene::GetBoxVertices(const Box& box, std::vector<glm::vec3>& outVertices) {
        glm::vec3 center = eigen2glm(box.center);
        Eigen::Matrix3f R = box.orientation.toRotationMatrix();
        glm::vec3 new_x = eigen2glm(R * Eigen::Vector3f(box.dim.x() / 2, 0.f, 0.f));
        glm::vec3 new_y = eigen2glm(R * Eigen::Vector3f(0.f, box.dim.y() / 2, 0.f));
        glm::vec3 new_z = eigen2glm(R * Eigen::Vector3f(0.f, 0.f, box.dim.z() / 2));
        outVertices.resize(8);
        outVertices[0] = center - new_x + new_y + new_z; outVertices[1] = center + new_x + new_y + new_z;
        outVertices[2] = center + new_x + new_y - new_z; outVertices[3] = center - new_x + new_y - new_z;
        outVertices[4] = center - new_x - new_y + new_z; outVertices[5] = center + new_x - new_y + new_z;
        outVertices[6] = center + new_x - new_y - new_z; outVertices[7] = center - new_x - new_y - new_z;
    }

    Common::CaseRenderResult CaseComplexScene::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        // 交互逻辑：使用 ForceManager 给场景中的方块施加外力
        std::vector<glm::vec3> candidateCenters;
        for (const auto& b : _dynamicBoxes) candidateCenters.push_back(eigen2glm(b.center));
        
        auto forceData = _forceManager.getForce(candidateCenters);
        glm::vec3 forceDelta = forceData.first;
        int hitIndex = forceData.second;
        
        if (glm::length(forceDelta) > 1e-6f && hitIndex >= 0 && hitIndex < _dynamicBoxes.size()) {
            // 直接施加冲量到质心，或者你可以修改给力臂产生旋转
            _dynamicBoxes[hitIndex].velocity += glm2eigen(forceDelta) / _dynamicBoxes[hitIndex].mass;
        }

        Advance(Engine::GetDeltaTime());

        _frame.Resize(desiredSize);
        _cameraManager.Update(_camera);
        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());

        gl_using(_frame);
        glEnable(GL_DEPTH_TEST);

        auto DrawBox = [&](const Box& b) {
            std::vector<glm::vec3> verts;
            GetBoxVertices(b, verts);
            auto span_bytes = Engine::make_span_bytes<glm::vec3>(verts);
            _program.GetUniforms().SetByName("u_Color", b.boxColor);
            _boxItem.UpdateVertexBuffer("position", span_bytes);
            _boxItem.Draw({ _program.Use() });
            _program.GetUniforms().SetByName("u_Color", glm::vec3(1.f, 1.f, 1.f));
            _lineItem.UpdateVertexBuffer("position", span_bytes);
            _lineItem.Draw({ _program.Use() });
        };

        for (const auto& b : _staticBoxes) DrawBox(b);
        for (const auto& b : _dynamicBoxes) DrawBox(b);

        return Common::CaseRenderResult { .Fixed = false, .Flipped = true, .Image = _frame.GetColorAttachment(), .ImageSize = desiredSize };
    }

    void CaseComplexScene::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);
        _forceManager.ProcessInput(_camera, pos); 
    }

} // namespace VCX::Labs::RigidBody