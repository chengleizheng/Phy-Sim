// CaseFlip.h
#pragma once
#include "FluidSimulator.h"
#include "BoundaryConditions.h"
#include "Labs/Common/ICase.h"
#include "Engine/GL/..."   // 和 Lab1 一致

namespace VCX::Labs::Fluid {

class CaseFlip : public Common::ICase {
public:
    CaseFlip();
    void                     OnSetupPropsUI() override;
    Common::CaseRenderResult OnRender(std::pair<uint32_t,uint32_t> desiredSize) override;
    void                     OnProcessInput(ImVec2 const& pos) override;

private:
    void Advance(float dt);  // 调 _sim.runSubStepLoop()

    FluidSimulator   _sim { 40, 40, 0.025f };  // 网格规模
    BoundaryConditions _bc;

    // UI 参数
    int   _numSubSteps = 3;
    float _dt          = 0.016f;

    // 渲染（和 Lab1 的 _boxItem 一样）
    Engine::GL::UniqueRenderFrame      _frame;
    Engine::GL::UniqueProgram          _program;
    Engine::GL::UniqueIndexedRenderItem _particleItem;
    Engine::GL::Camera                  _camera;
    Engine::GL::CameraManager           _cameraManager;
};

} // namespace