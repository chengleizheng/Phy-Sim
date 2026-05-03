#pragma once

#include "FluidSimulator.h"
#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Engine/Sphere.h"
#include "Labs/Common/ForceManager.h"
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
    float          _dt          = 0.007f;
    int            _numSubSteps = 1;

    // ── 渲染（fluid.vert/frag + 球体实例化）──
    Engine::GL::UniqueRenderFrame       _frame;
    Engine::GL::UniqueProgram           _program;
    Engine::Camera                      _camera { .Eye = glm::vec3(1.5f, 1.0f, 1.5f) };
    Common::OrbitCameraManager          _cameraManager;
    Common::ForceManager                _forceManager;      // 鼠标拖拽障碍球

    float                               _R = 0.15f;         // 障碍物半径 (必须在 _obstacleSphere 之前)
    Engine::Model                       _sphere;            // 球体几何
    std::vector<glm::vec3>              _particleOffsets;   // 每帧构建
    std::vector<glm::vec3>              _particleColors;    // 每帧构建
    Engine::Model                       _obstacleSphere;    // 障碍物几何

    Engine::GL::UniqueProgram          _lineprogram;   // flat shader
    Engine::GL::UniqueIndexedRenderItem _BoundaryItem; // Lines
    std::vector<glm::vec3>             _boundaryVerts;

    Labs::Rendering::SceneObject        _sceneObject { 1 }; // PassConstants 光照
    int                                 _colorMode = 1;     // 0=速度 1=密度 2=压强
};

} // namespace VCX::Labs::Fluid
