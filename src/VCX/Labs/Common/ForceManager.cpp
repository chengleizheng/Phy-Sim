#include <imgui_internal.h>
#include <iostream>
#include <vector>

#include "Engine/app.h"
#include "Labs/Common/ForceManager.h"

static bool intersectRayPlane(const glm::vec3& rayOrigin, const glm::vec3& rayDirection, const glm::vec3& planeNormal, const glm::vec3& planePoint, glm::vec3& intersectionPoint) {
    //calculate the intersection of the ray with the plane
    float denom = glm::dot(planeNormal, rayDirection);
    if (glm::abs(denom) > 1e-6f) 
    { // Ensure the ray is not parallel to the plane
        float t = glm::dot(planePoint - rayOrigin, planeNormal) / denom;
        if (t >= 0) { // Check if the intersection is in the direction of the ray
            intersectionPoint = rayOrigin + t * rayDirection;
            return true;
        }
    }
    else
    {
        return false; // No intersection, the ray is parallel to the plane
    }
}

glm::vec3 ScreenPointToWorldRay(const Engine::Camera& camera, const ImVec2& screenPos, const std::pair<std::uint32_t, std::uint32_t>& viewportSize) {
    // Convert screen coordinates to normalized device coordinates (NDC)
    
}
namespace VCX::Labs::Common {

}