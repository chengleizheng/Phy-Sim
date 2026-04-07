#pragma once

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/1-RigidBody/Box.h"
#include <fcl/fcl.h>

namespace VCX::Labs::RigidBody {

    class CaseNewtonPendulum : public Common::ICase {
    public:
        CaseNewtonPendulum();

        virtual std::string_view const GetName() override { return "Bonus 1: Newton Pendulum"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

        void ResetScene();
        void Advance(float timeDelta);

    private:
        void ProcessCollisions();
        void ApplyImpulse(Box& boxA, Box& boxB, const Eigen::Vector3f& p, const Eigen::Vector3f& n, float depth);
        void ApplyPositionCorrection(Box& boxA, Box& boxB, const Eigen::Vector3f& n, float depth);

        void GetBoxVertices(const Box& box, std::vector<glm::vec3>& outVertices);

        Engine::GL::UniqueProgram           _program;
        Engine::GL::UniqueRenderFrame       _frame;
        Engine::Camera                      _camera { .Eye = glm::vec3(0, 5, 15) };
        Common::OrbitCameraManager          _cameraManager;
        Engine::GL::UniqueIndexedRenderItem _boxItem;
        Engine::GL::UniqueIndexedRenderItem _lineItem;

        std::vector<Box> _dynamicBoxes;
        std::vector<Box> _staticBoxes;

        // 物理参数
        bool  _pause = true;
        bool  _enableSorting = true;
        int   _solverIterations = 10;
        float _restitution = 1.0f; // 完全弹性碰撞是牛顿摆的关键
        float _linearDamping = 1.0f; // 无阻力
    };
}