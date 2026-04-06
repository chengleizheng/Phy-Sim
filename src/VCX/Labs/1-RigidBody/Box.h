#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glm/glm.hpp>

namespace VCX::Labs::RigidBody {

    struct Box {
    public:
        Box(Eigen::Vector3f dim={ 1.f, 2.f, 3.f}, Eigen::Vector3f center={0.f,0.f,0.f},Eigen::Quaternionf orientation={1.f,0.f,0.f,0.f}, float mass=1.f) {
            this->dim = dim;
            this->center = center;
            this->orientation = orientation;
            this->mass = mass;
        };

        Eigen::Matrix3f GetInertiaMatrix() {
            // Using the formula from the image to calculate the inertia matrix Iref
            Eigen::Matrix3f Iref;
            float m = mass;
            float w = dim.x();
            float h = dim.y();
            float d = dim.z();
        
            Iref << 1.0f / 12.0f * m * (h * h + d * d), 0, 0,
                 0, 1.0f / 12.0f * m * (d * d + w * w), 0,
                 0, 0, 1.0f / 12.0f * m * (w * w + h * h);
        
            // Rotate the inertia matrix according to the current orientation of the box
            Eigen::Matrix3f R = orientation.toRotationMatrix();
            Eigen::Matrix3f inertiaMatrix = R * Iref * R.transpose();

            return inertiaMatrix;
        }


        glm::vec3                           boxColor { 121.0f / 255, 207.0f / 255, 171.0f / 255 };
        Eigen::Vector3f                     dim { 1.f, 2.f, 3.f };
        Eigen::Quaternionf                  orientation {1.0f, 0.0f, 0.0f, 0.0f}; // Quaternion for orientation
        Eigen::Vector3f                     angularVelocity {0.0f, 1.0f, 0.0f}; // Angular velocity
        Eigen::Vector3f                     velocity { 0.f, 0.f, 0.f };
        Eigen::Vector3f                     center { 0.f, 0.f, 0.f };

        int restingFrames{0}; // 连续静止帧数计数器
        
        float                               mass{1.f};
    };
} // namespace VCX::Labs::RigidBody