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

glm::vec3 ScreenPointToWorldRay(const glm::vec2& screenPoint, const VCX::Engine::Camera& camera, const ImVec2& windowSize) {
    // Convert screen coordinates to normalized device coordinates (NDC)
    // NDC range [-1, 1]
    float ndcX = (2.0f * screenPoint.x) / windowSize.x - 1.0f;
    float ndcY = 1.0f - (2.0f * screenPoint.y) / windowSize.y;
    glm::vec4 clipSpacePos(ndcX, ndcY, -1.0f, 1.0f);

    // Transform to world space
    glm::mat4 invProjMatrix = glm::inverse(camera.GetProjectionMatrix(windowSize.x / windowSize.y));
    glm::mat4 invViewMatrix = glm::inverse(camera.GetViewMatrix());
    glm::vec4 viewSpacePos = invProjMatrix * clipSpacePos;
    viewSpacePos.z = -1.0f; 
    // Set to -1 for a forward-facing ray
    viewSpacePos.w = 0.0f; 
    // Set to 0 for a direction vector
    glm::vec4 worldSpacePos = invViewMatrix * viewSpacePos;

    // Normalize the direction
    glm::vec3 rayDirection = glm::normalize(glm::vec3(worldSpacePos));
    return rayDirection;
}

glm::vec3 NearestPointOnRayToCubeCenter(const glm::vec3& rayOrigin, const glm::vec3& rayDirection, const glm::vec3& cubeCenter) {
    // Calculate the vector from the ray origin to the cube center
    glm::vec3 toCube = cubeCenter - rayOrigin;

    // Project this vector onto the ray direction
    float t = glm::dot(toCube, rayDirection);
    glm::vec3 nearestPointOnRay = rayOrigin + t * rayDirection;
    return nearestPointOnRay;
}

int NearestPointToRay(const glm::vec3& rayOrigin, const glm::vec3& rayDirection, const std::vector<glm::vec3>& candidatePoints) {
    int nearestPoint = 0;
    float minDistance = std::numeric_limits<float>::max();

    for (int i = 0; i < (int)candidatePoints.size(); ++i) {
        glm::vec3 nearestPointOnRay = NearestPointOnRayToCubeCenter(rayOrigin, rayDirection, candidatePoints[i]);
        float distance = glm::length(nearestPointOnRay - candidatePoints[i]);
        if (distance < minDistance) {
            minDistance = distance;
            nearestPoint = i;
        }
    }
    return nearestPoint;
}


static void printVec3(const glm::vec3& vec, const std::string& label = "") {
    if (!label.empty()) {
        std::cout << label << ": ";
    }
    std::cout << "(" << vec.x << ", " << vec.y << ", " << vec.z << ")" << std::endl;
}

namespace VCX::Labs::Common {
    void ForceManager::ProcessInput(Engine::Camera& camera, ImVec2 const &mousePosition) {
        auto window = ImGui::GetCurrentWindow();
        ImGuiIO const& io = ImGui::GetIO();
        bool anyHeld = false;
        bool hover = false;
        ImGui::ButtonBehavior(window->Rect(), window->GetID("##io"), &hover, &anyHeld);
        bool leftHeld = anyHeld && ImGui::IsMouseDown(ImGuiMouseButton_Left);
        ImVec2 const delta = io.MouseDelta;
        bool moving = (delta.x != 0.f || delta.y != 0.f) && hover;
        bool altKey = io.KeyAlt;

        float heightNorm = 1.f / window->Rect().GetHeight();

        bool applyingForce = moving && altKey && leftHeld;

        if (applyingForce) {
            glm::vec3 direction = camera.Target - camera.Eye;
            float targetDistance = glm::length(direction);
            glm::quat q = glm::quatLookAt(direction / targetDistance, camera.Up);

            float forceX = -delta.x * heightNorm;
            float forceY = delta.y * heightNorm;

            // We use only clientHeight here so aspect ratio does not distort speed
            _deltaForce = -(q * glm::vec3(2 * forceX * ForceScale * targetDistance, 2 * forceY * ForceScale * targetDistance, 0.f));

            // Calculate the ray direction from the screen point
            ImVec2 windowSize = ImVec2(window->Rect().GetWidth(), window->Rect().GetHeight());
            glm::vec2 screenPoint = glm::vec2(io.MousePos.x - window->Rect().Min.x, io.MousePos.y - window->Rect().Min.y);
            glm::vec3 rayDirection = ScreenPointToWorldRay(screenPoint, camera, windowSize);
            _camera = camera;
            _rayDirection = rayDirection;

        } else {
            _deltaForce = glm::vec3(0.f);
        }
    }

    glm::vec3 ForceManager::getForce() {
        return _deltaForce;
    }

    std::pair<glm::vec3,glm::vec3> ForceManager::getForce(glm::vec3 cubeCenter) {
        return std::pair<glm::vec3,glm::vec3>(_deltaForce,NearestPointOnRayToCubeCenter(_camera.Eye, _rayDirection, cubeCenter));
    }

    std::pair<glm::vec3,int> ForceManager::getForce(std::vector<glm::vec3> candidatePoints) {
        return std::pair<glm::vec3,int>(_deltaForce,NearestPointToRay(_camera.Eye, _rayDirection, candidatePoints));

    }


} // namespace VCX::Labs::Common