#pragma once

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/Common/ForceManager.h"
#include "Labs/1-RigidBody/Box.h"

#include <fcl/narrowphase/collision.h>
#include <fcl/geometry/shape/box.h>

namespace VCX::Labs::RigidBody {

    class CaseTwoBodies : public Common::ICase {
    public:
        CaseTwoBodies();

        virtual std::string_view const GetName() override { return "Impulse-Based Collision"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;
        
        void Advance(float timeDelta);
    
    private:
        void ProcessCollision();
        void ApplyImpulse(Box& boxA, Box& boxB, const Eigen::Vector3f& p, const Eigen::Vector3f& n, float depth);

        void GetBoxVertices(const Box& box, std::vector<glm::vec3>& vertices);
        
        Engine::GL::UniqueProgram           _program;
        Engine::GL::UniqueRenderFrame       _frame;
        Engine::Camera                      _camera { .Eye = glm::vec3(0, 5, 8) };
        Common::OrbitCameraManager          _cameraManager;

        Engine::GL::UniqueIndexedRenderItem _boxItem;  // render the box
        Engine::GL::UniqueIndexedRenderItem _lineItem; // render line on box

        Common::ForceManager                _forceManager;

        Box                                 _box0;
        Box                                 _box1;


        glm::vec3                           _boxColor { 0.75f, 0.92f, 1.0f };

        float                               _restitution { 0.8f }; // Coefficient of restitution for collision response
        bool                                _pause { false }; // Pause the simulation to inspect the collision state
    };
} // namespace VCX::Labs::RigidBody