#pragma once

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/Common/ForceManager.h"
#include "TetMesh.h"
#include "FEMIntegrator.h"

namespace VCX::Labs::FEM {

class CaseFEMSoftBody : public Common::ICase {
public:
    CaseFEMSoftBody();

    virtual std::string_view const GetName() override { return "FEM Soft Body"; }

    virtual void                     OnSetupPropsUI() override;
    virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
    virtual void                     OnProcessInput(ImVec2 const & pos) override;

private:
    void Advance(float dt);
    void ApplyMouseForce();
    void RebuildMesh();
    void ResetSimulation();

    Engine::GL::UniqueProgram           _program;
    Engine::GL::UniqueRenderFrame       _frame;
    Engine::GL::UniqueIndexedRenderItem _surfaceItem;
    Engine::GL::UniqueIndexedRenderItem _wireItem;

    Engine::Camera             _camera { .Eye = glm::vec3(3, 2, 5), .Target = glm::vec3(0, 1.5, 0) };
    Common::OrbitCameraManager _cameraManager;
    Common::ForceManager       _forceManager;

    TetMesh        _mesh;
    FEMIntegrator  _integrator;

    float _lambda    = 1000.0f;
    float _mu        = 500.0f;
    float _totalMass = 1.0f;
    float _damping   = 2.0f;
    glm::vec3 _gravity { 0.0f, -9.8f, 0.0f };

    glm::vec3 _surfaceColor { 0.2f, 0.6f, 0.8f };
    bool      _showWireframe = true;

    bool _fixTopFace = true;

    int _gridResX = 4;
    int _gridResY = 12;
    int _gridResZ = 4;

    glm::vec3 _beamOrigin { -0.5f, 0.0f, -0.5f };
    glm::vec3 _beamSize   { 1.0f, 3.0f, 1.0f };

    float _floorY = -0.05f;

    bool _needsRebuild = false;
};

} // namespace VCX::Labs::FEM
