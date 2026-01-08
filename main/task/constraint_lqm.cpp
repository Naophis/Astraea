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
      return 0.0f;
    }

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
    
    // Adaptive learning rate scaled by system gain to prevent divergence
    // When b is large, gradients are large, so we need smaller learning rate
    float base_lr = 0.01f;
    float learning_rate = base_lr / (1.0f + p.b * 0.1f); // Scale down with b
    // Manual clamp for C++14 compatibility
    if (learning_rate < 0.0001f) learning_rate = 0.0001f;
    if (learning_rate > 0.01f) learning_rate = 0.01f;

    // Simple fixed step size is risky, but for this specific mass/inertia it might work.
    // Better: Coordinate Descent (Gauss-Seidel) if we built H.
    // For standard "Lightweight" on MCU, usually we precompute H and g.
    // But H depends on N. We assume N is fixed.

    // Let's implement a very simple Forward-Backward rollout for gradient.

    int iter = p.max_iterations;
    if (iter <= 0) iter = 5; // Default small count

    // Safety check on input state
    if (std::abs(x0.theta_error) > 3.14f || std::abs(x0.omega_error) > 100.0f) {
        return 0.0f; // Invalid state, return zero control
    }

    for (int k = 0; k < iter; ++k) {
        // Forward pass: Compute states
        std::vector<State> x_seq(p.horizon + 1);
        x_seq[0] = x0;
        float dt = p.dt;
        float dt2 = dt * dt * 0.5f;
        
        for (int t = 0; t < p.horizon; ++t) {
            float u = u_sequence[t];
            float total_acc = (u + x_seq[t].d) * p.b;
            x_seq[t+1].theta_error = x_seq[t].theta_error + x_seq[t].omega_error * dt + total_acc * dt2;
            x_seq[t+1].omega_error = x_seq[t].omega_error + total_acc * dt;
            x_seq[t+1].d = x_seq[t].d; // Constant disturbance model

            // Clip predicted states to prevent numerical explosion
            if (x_seq[t+1].theta_error > 3.14f) x_seq[t+1].theta_error = 3.14f;
            if (x_seq[t+1].theta_error < -3.14f) x_seq[t+1].theta_error = -3.14f;
            if (x_seq[t+1].omega_error > 100.0f) x_seq[t+1].omega_error = 100.0f;
            if (x_seq[t+1].omega_error < -100.0f) x_seq[t+1].omega_error = -100.0f;
        }

        // Backward pass: Compute gradients (via costates)
        // Terminal cost: P*x_N (using same weights as stage cost for simplicity)
        State lambda_next;
        lambda_next.theta_error = 2 * p.q_ang * 10.0f * x_seq[p.horizon].theta_error; // 10x weight at terminal
        lambda_next.omega_error = 2 * p.q_vel * 10.0f * x_seq[p.horizon].omega_error;
        lambda_next.d = 0;
        
        for (int t = p.horizon - 1; t >= 0; --t) {
            // Costate equation: lambda_k = Q*x_k + A'*lambda_{k+1}
            State lambda;
            lambda.theta_error = 2 * p.q_ang * x_seq[t+1].theta_error + lambda_next.theta_error * 1.0f; // A(1,1)=1
            lambda.omega_error = 2 * p.q_vel * x_seq[t+1].omega_error + lambda_next.theta_error * dt + lambda_next.omega_error * 1.0f; 

            // Gradient w.r.t u_t: dJ/du = 2*R*u + B'*lambda_{k+1}
            float grad = 2 * p.r * u_sequence[t] + p.b * (lambda_next.theta_error * dt2 + lambda_next.omega_error * dt);

            // Strong gradient clipping for stability (scale with b)
            float grad_limit = 10.0f / (1.0f + p.b * 0.01f);
            if (grad > grad_limit) grad = grad_limit;
            if (grad < -grad_limit) grad = -grad_limit;

            // Update u with conservative step
            float u_new = u_sequence[t] - learning_rate * grad;

            // Project (Clamp) before updating
            if (u_new > u_max) u_new = u_max;
            if (u_new < u_min) u_new = u_min;
            u_sequence[t] = u_new;
            
            lambda_next = lambda;
        }
    }

    return u_sequence[0];
}
