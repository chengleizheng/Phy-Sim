// CaseFlip.cpp 核心部分

void CaseFlip::OnSetupPropsUI() {
    if (ImGui::CollapsingHeader("Simulation", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SliderFloat("dt",        &_dt,              0.001f, 0.05f);
        ImGui::SliderFloat("flipRatio", &_sim.flipRatio,   0.f,    1.f);   // 0=PIC 1=FLIP
        ImGui::SliderInt  ("subSteps",  &_numSubSteps,     1,      10);
        ImGui::SliderInt  ("solverIter",&_sim.solverIter,  1,      200);
        if (ImGui::Button("Reset")) { /* 重新初始化粒子 */ }
    }
}

void CaseFlip::Advance(float dt) {
    // 整个物理更新就这一行
    _sim.runSubStepLoop(_numSubSteps, dt / _numSubSteps, _bc);
}

void CaseFlip::OnRender(std::pair<uint32_t,uint32_t> desiredSize) {
    Advance(Engine::GetDeltaTime());   // 和 Lab1 一样先推进物理

    // 把 _sim.particles 的位置拷贝成 glm::vec3 数组，更新到 _particleItem
    // 其余渲染逻辑和 Lab1 的 _boxItem.Draw() 一致
}