function v = fcn(q, q_dot)
% LQR state feedback about the equilibrium: v = -K (x - x_eq)
% Copy of the "transformation" MATLAB Function block in
% simulink/underactuated_manipulator.slx. K comes from matlab/lqr_gain_calculation.m.

q1e = 1.0;
q2e = pi/2 - 1.0;

x = [q(1); q(2); q_dot(1); q_dot(2)];
x_eq = [q1e; q2e; 0; 0];
K = [268.4489  257.9608  206.6531  184.9319];
v = -K * (x - x_eq);

end
