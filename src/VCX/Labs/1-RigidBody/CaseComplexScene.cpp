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

    // 定义水晶色 (淡青色)
    glm::vec3 crystalColor(0.75f, 0.92f, 1.0f);
    // 定义深一点的边界线颜色（可选，用于辅助视觉）
    
    float floorSize = 15.0f;
    float wallHeight = 15.0f;
    float thickness = 1.5f;

    // 1. 地板：位于地面以下
    Box floor(Eigen::Vector3f(floorSize, thickness, floorSize), 
              Eigen::Vector3f(0.f, -thickness/2.0f, 0.f), 
              Eigen::Quaternionf::Identity(), 0.f);
    floor.boxColor = crystalColor;
    _staticBoxes.push_back(floor);

    // 2. 左侧墙体：高度增加，位置根据高度重新计算
    // 中心点 Y = wallHeight / 2
    Box wallLeft(Eigen::Vector3f(thickness, wallHeight, floorSize), 
                 Eigen::Vector3f(-floorSize/2.0f, wallHeight/2.0f, 0.f), 
                 Eigen::Quaternionf::Identity(), 0.f);
    wallLeft.boxColor = crystalColor * 0.9f; // 稍微暗一点点，增加层次感
    _staticBoxes.push_back(wallLeft);

    // 3. 右侧墙体
    Box wallRight(Eigen::Vector3f(thickness, wallHeight, floorSize), 
                  Eigen::Vector3f(floorSize/2.0f, wallHeight/2.0f, 0.f), 
                  Eigen::Quaternionf::Identity(), 0.f);
    wallRight.boxColor = crystalColor * 0.9f;
    _staticBoxes.push_back(wallRight);

    // 4. 后方墙体（可选，为了美观可以加上）
    Box wallBack(Eigen::Vector3f(floorSize, wallHeight, thickness), 
                 Eigen::Vector3f(0.f, wallHeight/2.0f, -floorSize/2.0f), 
                 Eigen::Quaternionf::Identity(), 0.f);
    wallBack.boxColor = crystalColor * 0.85f;
    _staticBoxes.push_back(wallBack);

    // 重新放置动态方块，让它们从更高的地方落下
    for (int i = 0; i < 6; ++i) {
        Box b;
        // ResetScene 里初始化动态盒子时加上：
        b.restingFrames = 0;
        b.mass = 1.0f;
        b.dim = Eigen::Vector3f(1.2f, 1.2f, 1.2f);
        b.center = Eigen::Vector3f(0.f, 5.0f + i * 2.5f, 0.f); // 垂直堆叠下落
        b.orientation = Eigen::Quaternionf(Eigen::AngleAxisf(0.4f * i, Eigen::Vector3f::UnitX()));
        b.boxColor = glm::vec3(1.0f, 0.4f + 0.1f * i, 0.4f); // 暖色方块与冷色背景对比
        _dynamicBoxes.push_back(b);
    }
}

    void CaseComplexScene::OnSetupPropsUI() {
        ImGui::Checkbox("Pause Simulation", &_pause);
        ImGui::SliderFloat("Gravity", &_gravity.y, -40.0f, 0.0f);
        ImGui::SliderFloat("Restitution", &_restitution, 0.0f, 1.0f);
        if (ImGui::Button("Reset Scene")) {
            ResetScene();
        }
    }

    void CaseComplexScene::Advance(float timeDelta) {
        if (_pause) return;

        // 1. 积分与外力应用
        for (auto& box : _dynamicBoxes) {
            if (box.restingFrames > 500  && &box != _activeBox) {
                box.velocity = Eigen::Vector3f::Zero();
                box.angularVelocity = Eigen::Vector3f::Zero();
                continue; // 位置和旋转也不更新
            }
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

    // 1. 碰撞检测阶段：收集所有有效碰撞点
    std::vector<ContactManifold> manifolds;

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
                    // 【关键修复 1】：放弃质心启发式，直接信任 FCL 的拓扑输出。
                    // FCL 保证法线从 o1(动态) 指向 o2(静态)。我们需要从静态 B 推开动态 A，所以必须取负！
                    avg_normal -= c.normal; 
                    max_depth = std::max(max_depth, c.penetration_depth);
                }
                avg_pos /= contacts.size();
                avg_normal.normalize();
                manifolds.push_back({&_dynamicBoxes[i], &_staticBoxes[j], avg_pos, avg_normal, max_depth});
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
                    // 同理，o1 是 i(A)，o2 是 j(B)，我们需要从 B 推开 A，取负。
                    avg_normal -= c.normal;
                    max_depth = std::max(max_depth, c.penetration_depth);
                }
                avg_pos /= contacts.size();
                avg_normal.normalize();
                manifolds.push_back({&_dynamicBoxes[i], &_dynamicBoxes[j], avg_pos, avg_normal, max_depth});
            }
        }
    }

    std::vector<bool> hasContact(_dynamicBoxes.size(), false);

for (auto& m : manifolds) {
    for (size_t i = 0; i < _dynamicBoxes.size(); ++i) {
        if (m.a == &_dynamicBoxes[i]) hasContact[i] = true;
    }
}

for (size_t i = 0; i < _dynamicBoxes.size(); ++i) {
    if (hasContact[i]) {
        _dynamicBoxes[i].restingFrames++;
    } else {
        _dynamicBoxes[i].restingFrames = 0; // 离开接触立刻重置
    }
}

    // 2. 速度求解阶段 (Velocity Solver)
    const int velocityIterations = 10;
    for (int iter = 0; iter < velocityIterations; ++iter) {
        for (auto& m : manifolds) {
            ApplyImpulse(*m.a, *m.b, m.pos, m.normal, m.depth);
        }
    }

    // 3. 位置修正阶段 (Position Solver)
    for (auto& m : manifolds) {
    bool aResting = (m.a->mass > 0.f && m.a->restingFrames > 5);
    bool bResting = (m.b->mass > 0.f && m.b->restingFrames > 5);
    
    // 静息接触只做极小补偿防止沉入，不做完整修正
    if (aResting || bResting) {
        const float percent = 0.05f; // 静息时大幅降低
        const float slop = 0.02f;    // 静息时允许更大容差
        float invMassA = m.a->mass > 0.0f ? 1.0f / m.a->mass : 0.0f;
        float invMassB = m.b->mass > 0.0f ? 1.0f / m.b->mass : 0.0f;
        Eigen::Vector3f correction = (std::max(m.depth - slop, 0.0f) / (invMassA + invMassB)) * percent * m.normal;
        if (invMassA > 0.f) m.a->center += correction * invMassA;
        if (invMassB > 0.f) m.b->center -= correction * invMassB;
    } else {
        ApplyPositionCorrection(*m.a, *m.b, m.normal, m.depth);
    }
}
}

void CaseComplexScene::ApplyImpulse(Box& boxA, Box& boxB, 
    const Eigen::Vector3f& p, const Eigen::Vector3f& n, float depth) {
    
    Eigen::Vector3f rA = p - boxA.center;
    Eigen::Vector3f rB = p - boxB.center;

    Eigen::Vector3f vA_p = boxA.velocity + boxA.angularVelocity.cross(rA);
    Eigen::Vector3f vB_p = boxB.velocity + boxB.angularVelocity.cross(rB);
    Eigen::Vector3f v_rel = vA_p - vB_p;

    float relVelAlongNormal = v_rel.dot(n);
    if (relVelAlongNormal > 0) return;

    // 接触超过 N 帧视为静息，直接归零弹性，彻底消除抖动
    float e = _restitution;
    bool isResting = (boxA.mass > 0.f && boxA.restingFrames > 5) || 
                     (boxB.mass > 0.f && boxB.restingFrames > 5);

    float invMassA = boxA.mass > 0.0f ? 1.0f / boxA.mass : 0.0f;
    float invMassB = boxB.mass > 0.0f ? 1.0f / boxB.mass : 0.0f;
    
    Eigen::Matrix3f invIa = Eigen::Matrix3f::Zero();
    if (boxA.mass > 0.0f) invIa = boxA.GetInertiaMatrix().inverse();
    Eigen::Matrix3f invIb = Eigen::Matrix3f::Zero();
    if (boxB.mass > 0.0f) invIb = boxB.GetInertiaMatrix().inverse();

    float termA = n.dot((invIa * (rA.cross(n))).cross(rA));
    float termB = n.dot((invIb * (rB.cross(n))).cross(rB));
    float denominator = invMassA + invMassB + termA + termB;
    if (denominator < 1e-6f) return;

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

void CaseComplexScene::ApplyPositionCorrection(Box& boxA, Box& boxB, const Eigen::Vector3f& n, float depth) {
    float invMassA = boxA.mass > 0.0f ? 1.0f / boxA.mass : 0.0f;
    float invMassB = boxB.mass > 0.0f ? 1.0f / boxB.mass : 0.0f;

    // 降低补偿比例，0.3f 能有效防止过冲 (Overshoot)
    const float percent = 0.3f; 
    const float slop = 0.005f;
    
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
            _dynamicBoxes[hitIndex].velocity += glm2eigen(forceDelta) / _dynamicBoxes[hitIndex].mass;
            _dynamicBoxes[hitIndex].restingFrames = 0;
            _activeBox = &_dynamicBoxes[hitIndex]; // 标记当前交互物体
        } else {
            _activeBox = nullptr; // 没有交互时清空
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