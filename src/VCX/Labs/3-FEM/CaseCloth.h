#pragma once

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/Common/ForceManager.h"
#include "ClothMesh.h"
#include "ClothFEMIntegrator.h"

namespace VCX::Labs::FEM {

class CaseCloth : public Common::ICase {
public:
    CaseCloth();

    virtual std::string_view const GetName() override { return "FEM Cloth"; }

    virtual void                     OnSetupPropsUI() override;
    virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
    virtual void                     OnProcessInput(ImVec2 const & pos) override;

private:
    void Advance(float dt);
    void ApplyMouseForce();
    void RebuildMesh();
    void ResetSimulation();
    void UpdateMaterialFromE();

    Engine::GL::UniqueProgram           _program;     // flat (wireframe)
    Engine::GL::UniqueProgram           _litProgram;  // lit (surface)
    Engine::GL::UniqueRenderFrame       _frame;
    Engine::GL::UniqueIndexedRenderItem _clothItem;   // lit surface (position + normal)
    Engine::GL::UniqueIndexedRenderItem _wireItem;    // flat wireframe

    Engine::Camera             _camera { .Eye = glm::vec3(0, 4, 3), .Target = glm::vec3(0, 0.5f, 0) };
    Common::OrbitCameraManager _cameraManager;
    Common::ForceManager       _forceManager;

    ClothMesh            _mesh;
    ClothFEMIntegrator   _integrator;

    // Material parameters
    float _E  = 50.0f;  // Young's modulus (Pa) — lower = softer, more stable
    float _nu = 0.3f;      // Poisson ratio

    float _totalMass = 0.1f; // kg (areal density ≈ mass/area = 0.1 kg/m²)
    float _damping   = 2.0f;
    glm::vec3 _gravity { 0.0f, -9.8f, 0.0f };

    // Appearance
    glm::vec3 _surfaceColor { 0.2f, 0.5f, 0.8f };
    bool      _showWireframe = true;

    // Lighting
    bool      _useLighting    = true;
    float     _lightIntensity = 1.0f;
    float     _ambientScale   = 0.2f;
    float     _shininess      = 32.0f;
    bool      _flatShading    = true;
    glm::vec3 _lightDir { 0.5f, 0.8f, 0.6f };

    // Mesh
    int   _gridResX   = 20;
    int   _gridResY   = 20;
    float _clothWidth  = 1.0f;
    float _clothHeight = 1.0f;
    float _initHeight  = 1.1f;
    bool  _pinCorners  = true;

    // Simulation
    float _floorY      = 0.0f;
    float _windForce   = 30.0f;
    int   _numSubsteps = 150;

    bool _needsRebuild = false;
};

} // namespace VCX::Labs::FEM
