#pragma once

#include "Engine/Camera.hpp"
#include "Engine/math.hpp"
#include "Labs/Common/ImGuiHelper.h"

namespace VCX::Labs::Common {
    class ForceManager {
        public:
            ForceManager() = default;

            bool isEnabled = true;
            float ForceScale = 1.0f; 

            void ProcessInput(Engine::Camera & camera, ImVec2 const & mousePosition);
            glm::vec3 getForce();   //just force vector
            std::pair<glm::vec3,glm::vec3> getForce(glm::vec3 cubeCenter);   // force with the point of application
            std::pair<glm::vec3,int> getForce(std::vector<glm::vec3> candidatePoints);    //apply to the closest point among candidate points
        
        private:
            glm::vec3 _deltaForce = glm::vec3 { 0.f };
            Engine::Camera _camera;
            glm::vec3 _rayDirection = glm::vec3 { 0.f };
        };
}// namespace VCX::Labs::Common