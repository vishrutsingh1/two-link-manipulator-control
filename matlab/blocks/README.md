# MATLAB Function block code

These `.m` files are **read-only copies** of the MATLAB Function blocks inside the Simulink models, so the controller code can be read on GitHub without opening MATLAB. The models in `../../simulink/` are the source of truth. If you change a block, copy the new code here too.

| File | Block in the model | What it does |
|------|--------------------|--------------|
| `fully_actuated/trajectory_generator.m` | `trajectory_generator` | Reference trajectories q_ref1(t), q_ref2(t) |
| `fully_actuated/outer_loop_transformation.m` | `transformation` | State feedback v = Kp·q + Kd·q̇ (the integral term Ki = 1320 is in Simulink blocks) |
| `fully_actuated/feedback_linearisation.m` | `Linearisation` | Computed-torque law τ = M(q)·v + C(q, q̇)q̇ + G(q) |
| `fully_actuated/manipulator_dynamics.m` | `manipulator_dynamics` | Nonlinear plant: q̈ = M⁻¹(τ + τ_d − Cq̇ − G) |
| `underactuated/trajectory_generator.m` | `trajectory_generator` | Constant equilibrium set-point |
| `underactuated/lqr_state_feedback.m` | `transformation` | LQR law v = −K(x − x_eq) |
| `underactuated/partial_feedback_linearisation.m` | `Linearisation` | τ₁ = α(x)·v + β(x), τ₂ = 0 |
| `underactuated/manipulator_dynamics.m` | `manipulator_dynamics` | Nonlinear plant with joint 2 unactuated |
