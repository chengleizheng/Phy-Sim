#pragma once
#include <Eigen/Dense>

// 边界类型
enum class BCType { Dirichlet, Neumann };

struct BoundaryConditions {
    BCType type = BCType::Neumann;
    Eigen::Vector3f minBound { 0.f, 0.f, 0.f };  
    Eigen::Vector3f maxBound { 1.f, 1.f, 1.f }; 
    Eigen::Vector3f wallVelocity { 0.f, 0.f, 0.f };
};