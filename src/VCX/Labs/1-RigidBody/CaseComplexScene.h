#pragma once

#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "Labs/Common/ForceManager.h"
#include "Labs/1-RigidBody/Box.h"

#include <fcl/narrowphase/collision.h>
#include <fcl/geometry/shape/box.h>
#include <vector>

namespace VCX::Labs::RigidBody {

    class CaseComplexScene : public Common::ICase {
    public:
        CaseComplexScene();

        virtual std::string_view const GetName() override { return "Complex Scene (Gravity & Walls)"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

        void Advance(float timeDelta);
        float                               _restitution { 0.4f }; // 堆叠场景建议降低恢复系数
        Box* _activeBox = nullptr; 

    private:
        struct ContactManifold {
            Box* a;
            Box* b;
            Eigen::Vector3f pos;
            Eigen::Vector3f normal;
            float depth;
        };
        void ProcessCollisions();
        void ApplyImpulse(Box& boxA, Box& boxB, const Eigen::Vector3f& p, const Eigen::Vector3f& n, float depth);
        void GetBoxVertices(const Box& box, std::vector<glm::vec3>& outVertices);
        void ApplyPositionCorrection(Box& boxA, Box& boxB, const Eigen::Vector3f& n, float depth);
        void ResetScene();

        Engine::GL::UniqueProgram           _program;
        Engine::GL::UniqueRenderFrame       _frame;
        Engine::Camera                      _camera { .Eye = glm::vec3(0, 8, 15) };
        Common::OrbitCameraManager          _cameraManager;
        
        Engine::GL::UniqueIndexedRenderItem _boxItem;  
        Engine::GL::UniqueIndexedRenderItem _lineItem; 
        
        Common::ForceManager                _forceManager;
        
        // 场景数据
        std::vector<Box>                    _dynamicBoxes;
        std::vector<Box>                    _staticBoxes; // 地板和墙壁
        
        // 物理参数
        glm::vec3                           _gravity { 0.f, -9.8f, 0.f };
        float                               _linearDamping { 0.99f };
        float                               _angularDamping { 0.97f };
        bool                                _pause { false };
    };
} // namespace VCX::Labs::RigidBody