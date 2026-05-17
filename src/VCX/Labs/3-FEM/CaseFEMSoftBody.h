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

    Engine::GL::UniqueProgram           _program;     // flat (wireframe)
    Engine::GL::UniqueProgram           _litProgram;  // lit (surface)
    Engine::GL::UniqueRenderFrame       _frame;
    Engine::GL::UniqueIndexedRenderItem _surfaceItem; // lit surface (position + normal)
    Engine::GL::UniqueIndexedRenderItem _wireItem;    // flat wireframe

    Engine::Camera             _camera { .Eye = glm::vec3(5, 3, 5), .Target = glm::vec3(0, 0.5, 0) };
    Common::OrbitCameraManager _cameraManager;
    Common::ForceManager       _forceManager;

    TetMesh        _mesh;
    FEMIntegrator  _integrator;

    float _lambda    = 300.0f;
    float _mu        = 50.0f;
    float _totalMass = 1.0f;
    float _damping   = 2.8f;
    glm::vec3 _gravity { 0.0f, -9.8f, 0.0f };

    glm::vec3 _surfaceColor { 0.2f, 0.6f, 0.8f };
    bool      _showWireframe = true;

    // lighting
    bool     _useLighting = true;
    float    _lightIntensity = 1.0f;
    float    _ambientScale = 0.30f;
    float    _shininess = 150.0f;
    bool     _flatShading = true;
    glm::vec3 _lightDir { 7.0f, 9.0f, 7.0f };

    bool _fixTopFace = false;

    int _gridResX = 12;
    int _gridResY = 4;
    int _gridResZ = 4;

    glm::vec3 _beamOrigin { -1.5f, 0.0f, -0.5f };
    glm::vec3 _beamSize   { 3.0f, 1.0f, 1.0f };

    float _floorY    = 0.0f;
    float _liftForce = 50.0f;
    int _numSubsteps = 100;

    bool _needsRebuild = false;
};

} // namespace VCX::Labs::FEM
