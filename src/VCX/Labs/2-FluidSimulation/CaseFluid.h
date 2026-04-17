#pragma once

#include "FluidSimulator.h"
#include "BoundaryConditions.h"
#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/Common/ForceManager.h"

namespace VCX::Labs::Fluid {

class CaseFluid : public Common::ICase {
public:
    CaseFluid();

    virtual std::string_view const GetName() override { return "FLIP Fluid Simulation"; }

    virtual void                     OnSetupPropsUI() override;
    virtual Common::CaseRenderResult OnRender(std::pair<uint32_t,uint32_t> desiredSize) override;
    virtual void                     OnProcessInput(ImVec2 const& pos) override;
    float _dt          = 0.016f;    //由于要求可调整时间步长，因此把它放在 Case 类里，UI 里调整这个值就行了
    float _flipRatio    = 0.95f;     //同样地，flipRatio 也放在 Case 类里

private:
    void Advance(float dt);  // 调 _sim.runSubStepLoop()

    FluidSimulator   _sim { 40, 40, 0.025f };  // 网格规模
    BoundaryConditions _bc;

    // UI 参数
    int   _numSubSteps = 3;

    // 渲染（和 Lab1 的 _boxItem 一样）
    Engine::GL::UniqueRenderFrame      _frame;
    Engine::GL::UniqueProgram          _program;
    Engine::GL::UniqueIndexedRenderItem _particleItem;
    Engine::GL::Camera                  _camera;
    Engine::GL::CameraManager           _cameraManager;
};

} // namespace VCX::Labs::Fluid