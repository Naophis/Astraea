#ifndef CONSTRAINT_LQM_HPP
#define CONSTRAINT_LQM_HPP

#include <vector>
#include <cmath>
#include <algorithm>

/**
 * @brief Lightweight Constraint Linear Quadratic "Model Predictive" Control
 * 
 * Simple implementation for 1-input LTI system:
 *    x(k+1) = A x(k) + B u(k)
 *    J = sum(x'Qx + u'Ru)
 *    u_min <= u <= u_max
 * 
 * Uses Projected Gradient Descent to solve the constrained QP.
 * Optimized for MCU (minimal dynamic allocation, fixed iteration count).
 */
class ConstraintLQM {
public:
    struct State {
        float theta_error;
        float omega_error;
        float d; // Disturbance
    };

    struct Parameters {
        float q_ang;
        float q_vel;
        float b;
        float r; 
        int horizon;
        float dt;
        int max_iterations; // For QP solver
    };

    ConstraintLQM();

    void initialize(const Parameters& params);
    float solve(const State& x0, float u_min, float u_max);

private:
    Parameters p;
    std::vector<float> u_sequence;
    bool initialized;
};

#endif // CONSTRAINT_LQM_HPP
