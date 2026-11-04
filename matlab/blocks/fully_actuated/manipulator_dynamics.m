function [q1_ddot, q2_ddot] = plant(q, q_dot, tau_in, td)
% Nonlinear two-link manipulator dynamics: q_ddot = M^-1 (tau + td - C - G)
% Copy of the "manipulator_dynamics" MATLAB Function block in
% simulink/fully_actuated_manipulator.slx.
%
% NOTE: in the saved model the line below sets the disturbance td to zero,
% which gives the no-disturbance case. Remove it to apply the disturbance
% from the Noise subsystem:
%   td1 = 2 sin(t) + 0.5 sin(200*pi*t),  td2 = cos(2t) + 0.5 sin(200*pi*t)

l1 = 1; l2 = 0.8;
m1 = 0.5; m2 = 1.5;
j1 = 5; j2 = 5;
g = 9.8;

m11 = m1*l1^2 + m2*(l1^2 + l2^2 + 2*l1*l2*cos(q(2))) + j1 + j2;
m12 = m2*(l2^2 + l1*l2*cos(q(2))) + j2;
m21 = m12;
m22 = m2*l2^2 + j2;
M = [m11, m12; m21, m22];

h = m2*l1*l2*sin(q(2));
C_vec = [-h*q_dot(2)^2 - 2*h*q_dot(1)*q_dot(2);
          h*q_dot(1)^2];

G1 = (m1 + m2)*l1*g*cos(q(1)) + m2*l2*g*cos(q(1) + q(2));
G2 = m2*l2*g*cos(q(1) + q(2));
G_vec = [G1; G2];

td = [0;0];   % disturbance switched off (see note above)

q2 = M \ (tau_in + td - C_vec - G_vec);

q1_ddot = q2(1);
q2_ddot = q2(2);

end
