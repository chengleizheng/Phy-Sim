#pragma once

#include "FluidSimulator.h"
#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Engine/Sphere.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/Scene/SceneObject.h"

namespace VCX::Labs::Fluid {

class CaseFluid : public Common::ICase {
public:
    CaseFluid();

    virtual std::string_view const GetName() override { return "FLIP Fluid Simulation"; }

    virtual void                     OnSetupPropsUI() override;
    virtual Common::CaseRenderResult OnRender(std::pair<uint32_t, uint32_t> desiredSize) override;
    virtual void                     OnProcessInput(ImVec2 const & pos) override;

private:
    void Advance();

    // ── 仿真 ──
    FluidSimulator _sim { 25, 25, 25, 1.0f / 25 };  // 网格 25³, 域 [0,1]³
    float          _dt          = 0.016f;
    int            _numSubSteps = 1;

    // ── 渲染（fluid.vert/frag + 球体实例化）──
    Engine::GL::UniqueRenderFrame       _frame;
    Engine::GL::UniqueProgram           _program;
    Engine::Camera                      _camera { .Eye = glm::vec3(1.5f, 1.0f, 1.5f) };
    Common::OrbitCameraManager          _cameraManager;

    Engine::Model                       _sphere;            // 球体几何
    std::vector<glm::vec3>              _particleOffsets;   // 每帧构建
    std::vector<glm::vec3>              _particleColors;    // 每帧构建

    Labs::Rendering::SceneObject        _sceneObject { 1 }; // PassConstants 光照
};

} // namespace VCX::Labs::Fluid
