#include "Labs/1-RigidBody/CaseNewtonPendulum.h"
#include "Labs/Common/ImGuiHelper.h"
#include "Engine/app.h"
#include <iostream>
#include <algorithm>

static glm::vec3 eigen2glm(const Eigen::Vector3f& eigenVec) {
    return glm::vec3(eigenVec.x(), eigenVec.y(), eigenVec.z());
}

static Eigen::Vector3f glm2eigen(const glm::vec3& glmVec) {
    return Eigen::Vector3f(glmVec.x, glmVec.y, glmVec.z);
}

namespace VCX::Labs::RigidBody {

    // 内部结构体，用于保存碰撞流形及排序参数
    struct ContactManifold {
        Box* a;
        Box* b;
        Eigen::Vector3f pos;
        Eigen::Vector3f normal;
        float depth;
        float approaching_vel; // 用于排序：记录碰撞初期的相对速度
    };

    CaseNewtonPendulum::CaseNewtonPendulum():
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

        ResetScene();
    }

    void CaseNewtonPendulum::ResetScene() {
        _dynamicBoxes.clear();
        _staticBoxes.clear();

        // 1. 构建无摩擦光滑地板
        Box floor(Eigen::Vector3f(20.f, 1.f, 5.f), Eigen::Vector3f(0.f, -0.5f, 0.f), Eigen::Quaternionf::Identity(), 0.f);
        floor.boxColor = glm::vec3(0.7f, 0.8f, 0.9f); // 冰蓝色地板
        _staticBoxes.push_back(floor);

        // 2. 构建牛顿摆方块阵列
        int numBoxes = 5;
        float boxSize = 1.0f;
        float startX = -((numBoxes - 1) * boxSize) / 2.0f;

        for (int i = 0; i < numBoxes; ++i) {
            Box b;
            b.mass = 1.0f;
            b.dim = Eigen::Vector3f(boxSize, boxSize, boxSize);
            // 相互紧贴，预留 0.001f 微小间隙防初始穿透
            b.center = Eigen::Vector3f(startX + i * (boxSize + 0.001f), boxSize / 2.0f, 0.f); 
            b.orientation = Eigen::Quaternionf::Identity();
            
            if (i == 0) {
                // 第一个方块作为撞击子，拉开距离并给予极高初速度
                b.center.x() -= 4.0f; 
                b.velocity = Eigen::Vector3f(8.0f, 0.f, 0.f); 
                b.boxColor = glm::vec3(1.0f, 0.3f, 0.3f); // 红色高亮显示
            } else {
                // 靶子方块静止
                b.velocity = Eigen::Vector3f::Zero();
                b.boxColor = glm::vec3(0.3f, 0.6f, 1.0f); // 蓝色
            }
            
            b.angularVelocity = Eigen::Vector3f::Zero();
            _dynamicBoxes.push_back(b);
        }
    }

    void CaseNewtonPendulum::OnSetupPropsUI() {
        ImGui::Checkbox("Pause Simulation", &_pause);
        ImGui::Spacing();
        
        ImGui::Text("Newton's Cradle Parameters:");
        ImGui::SliderInt("Solver Iterations", &_solverIterations, 1, 50);
        ImGui::SliderFloat("Restitution", &_restitution, 0.0f, 1.0f);
        
        // 绑定到类成员变量 _enableSorting
        ImGui::Checkbox("Enable Contact Sorting (Bonus 1)", &_enableSorting);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Sort contacts by velocity to simulate momentum propagation.");
        }

        ImGui::Spacing();
        if (ImGui::Button("Reset Pendulum")) {
            ResetScene();
        }
    }

    void CaseNewtonPendulum::Advance(float timeDelta) {
        if (_pause) return;

        // 1. 施加重力 (这里是水平测试，主要靠地板支撑)
        Eigen::Vector3f gravity(0.f, -9.8f, 0.f);
        for (auto& box : _dynamicBoxes) {
            box.velocity += gravity * timeDelta;
            // 牛顿摆应尽量无阻尼，以观察动能完美传递
            box.velocity *= _linearDamping; 
        }

        // 2. 碰撞检测与求解 (冲量法更新速度)
        ProcessCollisions();

        // 3. 半隐式位置更新
        for (auto& box : _dynamicBoxes) {
            box.center += timeDelta * box.velocity;
            
            // 为了演示理想的 1D 牛顿摆，我们在此强行消除旋转动能的耗散
            box.angularVelocity = Eigen::Vector3f::Zero();
            box.orientation = Eigen::Quaternionf::Identity();
        }
    }

    void CaseNewtonPendulum::ProcessCollisions() {
        using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<float>>;
        fcl::CollisionRequest<float> request(8, true);

        std::vector<fcl::CollisionObject<float>> dynObjs;
        for (const auto& box : _dynamicBoxes) {
            CollisionGeometryPtr_t shape(new fcl::Box<float>(box.dim.x(), box.dim.y(), box.dim.z()));
            fcl::Transform3f tf(fcl::Translation3f(box.center) * box.orientation);
            dynObjs.emplace_back(shape, tf);
        }

        std::vector<fcl::CollisionObject<float>> statObjs;
        for (const auto& box : _staticBoxes) {
            CollisionGeometryPtr_t shape(new fcl::Box<float>(box.dim.x(), box.dim.y(), box.dim.z()));
            fcl::Transform3f tf(fcl::Translation3f(box.center) * box.orientation);
            statObjs.emplace_back(shape, tf);
        }

        std::vector<ContactManifold> manifolds;

        // A. 动态 vs 静态 (地板)
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
                        Eigen::Vector3f n = -c.normal; // 静态推开动态
                        avg_normal += n;
                        max_depth = std::max(max_depth, c.penetration_depth);
                    }
                    avg_pos /= contacts.size();
                    avg_normal.normalize();
                    
                    // 计算接近速度
                    Eigen::Vector3f v_rel = _dynamicBoxes[i].velocity - _staticBoxes[j].velocity;
                    float app_vel = v_rel.dot(avg_normal);

                    manifolds.push_back({&_dynamicBoxes[i], &_staticBoxes[j], avg_pos, avg_normal, max_depth, app_vel});
                }
            }
        }

        // B. 动态 vs 动态 (牛顿摆的方块相互碰撞)
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
                        Eigen::Vector3f n = -c.normal; // B 推开 A
                        avg_normal += n;
                        max_depth = std::max(max_depth, c.penetration_depth);
                    }
                    avg_pos /= contacts.size();
                    avg_normal.normalize();

                    // 计算相对接近速度
                    Eigen::Vector3f v_rel = _dynamicBoxes[i].velocity - _dynamicBoxes[j].velocity;
                    float app_vel = v_rel.dot(avg_normal);

                    manifolds.push_back({&_dynamicBoxes[i], &_dynamicBoxes[j], avg_pos, avg_normal, max_depth, app_vel});
                }
            }
        }

        // ==========================================
        // BONUS 1 探究：接触点排序策略 (Shock Propagation / Contact Sorting)
        // 串行冲量法默认会因遍历顺序导致动能被抹平。
        // 将撞击最剧烈（相对速度最负）的接触点放在最前面求解，能显著提高冲量传导的精度。
        // ==========================================
        if (_enableSorting) {
            std::sort(manifolds.begin(), manifolds.end(), [](const ContactManifold& m1, const ContactManifold& m2) {
                return m1.approaching_vel < m2.approaching_vel;
            });
        }

        // 速度迭代求解
        for (int iter = 0; iter < _solverIterations; ++iter) {
            for (auto& m : manifolds) {
                ApplyImpulse(*m.a, *m.b, m.pos, m.normal, m.depth);
            }
        }

        // 位置补偿
        for (auto& m : manifolds) {
            ApplyPositionCorrection(*m.a, *m.b, m.normal, m.depth);
        }
    }

    void CaseNewtonPendulum::ApplyImpulse(Box& boxA, Box& boxB, const Eigen::Vector3f& p, const Eigen::Vector3f& n, float depth) {
        Eigen::Vector3f rA = p - boxA.center;
        Eigen::Vector3f rB = p - boxB.center;

        Eigen::Vector3f vA_p = boxA.velocity + boxA.angularVelocity.cross(rA);
        Eigen::Vector3f vB_p = boxB.velocity + boxB.angularVelocity.cross(rB);
        Eigen::Vector3f v_rel = vA_p - vB_p;

        float relVelAlongNormal = v_rel.dot(n);
        if (relVelAlongNormal > 0) return; // 分离状态跳过

        float e = _restitution;
        // 静息接触处理
        if (relVelAlongNormal > -0.5f) {
            e = 0.0f;
        }

        float invMassA = boxA.mass > 0.0f ? 1.0f / boxA.mass : 0.0f;
        float invMassB = boxB.mass > 0.0f ? 1.0f / boxB.mass : 0.0f;
        
        Eigen::Matrix3f invIa = Eigen::Matrix3f::Zero();
        if (boxA.mass > 0.0f) invIa = boxA.GetInertiaMatrix().inverse();
        
        Eigen::Matrix3f invIb = Eigen::Matrix3f::Zero();
        if (boxB.mass > 0.0f) invIb = boxB.GetInertiaMatrix().inverse();

        float termA = n.dot( (invIa * (rA.cross(n))).cross(rA) );
        float termB = n.dot( (invIb * (rB.cross(n))).cross(rB) );
        
        float denominator = invMassA + invMassB + termA + termB;
        if (denominator < 1e-6f) return; 

        // 无摩擦的冲量大小
        float j = -(1.0f + e) * relVelAlongNormal / denominator;
        Eigen::Vector3f J = j * n;

        if (invMassA > 0.f) {
            boxA.velocity += J * invMassA;
            boxA.angularVelocity += invIa * rA.cross(J);
        }
        if (invMassB > 0.f) {
            boxB.velocity -= J * invMassB;
            boxB.angularVelocity -= invIb * rB.cross(J);
        }
    }

    void CaseNewtonPendulum::ApplyPositionCorrection(Box& boxA, Box& boxB, const Eigen::Vector3f& n, float depth) {
        float invMassA = boxA.mass > 0.0f ? 1.0f / boxA.mass : 0.0f;
        float invMassB = boxB.mass > 0.0f ? 1.0f / boxB.mass : 0.0f;

        const float percent = 0.2f; 
        const float slop = 0.01f;
        
        Eigen::Vector3f correction = (std::max(depth - slop, 0.0f) / (invMassA + invMassB)) * percent * n;
        
        if (invMassA > 0.f) boxA.center += correction * invMassA;
        if (invMassB > 0.f) boxB.center -= correction * invMassB;
    }

    void CaseNewtonPendulum::GetBoxVertices(const Box& box, std::vector<glm::vec3>& outVertices) {
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

    Common::CaseRenderResult CaseNewtonPendulum::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        Advance(Engine::GetDeltaTime());

        _frame.Resize(desiredSize);
        _cameraManager.Update(_camera);
        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());

        gl_using(_frame);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LINE_SMOOTH);

        auto DrawBox = [&](const Box& b) {
            std::vector<glm::vec3> verts;
            GetBoxVertices(b, verts);
            auto span_bytes = Engine::make_span_bytes<glm::vec3>(verts);
            
            _program.GetUniforms().SetByName("u_Color", b.boxColor);
            _boxItem.UpdateVertexBuffer("position", span_bytes);
            _boxItem.Draw({ _program.Use() });
            
            _program.GetUniforms().SetByName("u_Color", glm::vec3(0.1f, 0.1f, 0.1f));
            _lineItem.UpdateVertexBuffer("position", span_bytes);
            _lineItem.Draw({ _program.Use() });
        };

        for (const auto& b : _staticBoxes) DrawBox(b);
        for (const auto& b : _dynamicBoxes) DrawBox(b);

        glDisable(GL_LINE_SMOOTH);

        return Common::CaseRenderResult { .Fixed = false, .Flipped = true, .Image = _frame.GetColorAttachment(), .ImageSize = desiredSize };
    }

    void CaseNewtonPendulum::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);
    }

} // namespace VCX::Labs::RigidBody