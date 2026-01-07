#include "include/constraint_lqm.hpp"
#include <iostream>

ConstraintLQM::ConstraintLQM() : initialized(false) {}

void ConstraintLQM::initialize(const Parameters& params) {
    p = params;
    initialized = true;
    
    // Pre-allocate or reset warm-start buffer
    u_sequence.assign(p.horizon, 0.0f);
}

float ConstraintLQM::solve(const State& x0, float u_min, float u_max) {
    if (!initialized || p.horizon <= 0) {
      // printf("horizon %d <= 0 or not initialized\n", p.horizon);
      return 0.0f;
    }

    // printf("check: ");
    // Model:
    // x = [theta_err; omega_err]
    // u = angular_accel * dt (or similar, depending on B)
    // A = [1, dt; 0, 1]
    // B = [0.5*dt*dt; dt] 
    // 
    // This is a simplified unicycle yaw model.
    // We will solve for U = [u0, ..., uN]
    
    // 1. Warm start: Shift previous solution
    for (size_t i = 0; i < p.horizon - 1; ++i) {
        u_sequence[i] = u_sequence[i+1];
    }
    u_sequence[p.horizon - 1] = 0.0f; // Fill end with zero

    // 2. Projected Gradient Descent (Fixed Iterations)
    // Since constructing the full Hessian H for N=20 is expensive (20x20 matrix),
    // we can compute gradients iteratively using adjoint (costate) equations.
    // or just implement a very small horizon (N=5~10) and unroll.
    
    // For efficiency, we will use a small horizon and gradient descent.
    // Gradient of J w.r.t u_k is: 2*R*u_k + 2*B^T * lambda_(k+1)
    // where lambda is the costate: lambda_k = Q*x_k + A^T*lambda_(k+1)
    
    float learning_rate = 0.001f; // Needs manual tuning or line search
    // Simple fixed step size is risky, but for this specific mass/inertia it might work.
    // Better: Coordinate Descent (Gauss-Seidel) if we built H.
    // For standard "Lightweight" on MCU, usually we precompute H and g.
    // But H depends on N. We assume N is fixed.
    
    // Let's implement a very simple Forward-Backward rollout for gradient.
    
    int iter = p.max_iterations;
    if (iter <= 0) iter = 5; // Default small count

    for (int k = 0; k < iter; ++k) {
        // Forward pass: Compute states
        std::vector<State> x_seq(p.horizon + 1);
        x_seq[0] = x0;
        float dt = p.dt;
        float dt2 = dt * dt * 0.5f;
        
        for (int t = 0; t < p.horizon; ++t) {
            float u = u_sequence[t];
            x_seq[t+1].theta_error = x_seq[t].theta_error + x_seq[t].omega_error * dt + u * dt2; 
            x_seq[t+1].omega_error = x_seq[t].omega_error + u * dt; // Approximating B*u as accel input
                                                                    // u here is likely "alpha * dt" ? 
                                                                    // Wait, if input is DUTY, B needs to map Duty->Alpha->Omega.
                                                                    // For simplicity, let's assume u is "control effort" ~ alpha.
        }

        // Backward pass: Compute gradients (via costates)
        State lambda_next = {0, 0}; // lambda_N = 0 (or terminal cost P*x_N)
        
        for (int t = p.horizon - 1; t >= 0; --t) {
            // Costate equation: lambda_k = Q*x_k + A'*lambda_{k+1}
            State lambda;
            lambda.theta_error = 2 * p.q_ang * x_seq[t+1].theta_error + lambda_next.theta_error * 1.0f; // A(1,1)=1
            lambda.omega_error = 2 * p.q_vel * x_seq[t+1].omega_error + lambda_next.theta_error * dt + lambda_next.omega_error * 1.0f; 

            // Gradient w.r.t u_t: dJ/du = 2*R*u + B'*lambda_{k+1}
            float grad = 2 * p.r * u_sequence[t] + (lambda_next.theta_error * dt2 + lambda_next.omega_error * dt);
            
            // Update u
            u_sequence[t] -= learning_rate * grad;
            
            // Project (Clamp)
            u_sequence[t] = std::clamp(u_sequence[t], u_min, u_max);
            
            lambda_next = lambda;
        }
    }
    
    return u_sequence[0];
}
