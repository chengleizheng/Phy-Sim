#pragma once
#include <Eigen/Dense>

// 边界类型
enum class BCType { Dirichlet, Neumann };

struct BoundaryConditions {
    BCType type = BCType::Neumann;
    // 边界盒：粒子必须在 [minBound, maxBound] 内
    Eigen::Vector2f minBound { 0.f, 0.f };
    Eigen::Vector2f maxBound { 1.f, 1.f };
    // 固体速度（Dirichlet时使用，一般为零）
    Eigen::Vector2f wallVelocity { 0.f, 0.f };
};