function [ddq_1, ddq_2] = plant(q, dq, tau)
% Nonlinear two-link manipulator dynamics with joint 2 unactuated (tau(2) = 0).
% Copy of the "manipulator_dynamics" MATLAB Function block in
% simulink/underactuated_manipulator.slx.

r1 = 1.0; r2 = 0.8;
m1 = 0.5; m2 = 1.5;
J1 = 5.0; J2 = 5.0;
g  = 9.8;

q1 = q(1); q2 = q(2);
dq1 = dq(1); dq2 = dq(2);
m11 = J1 + J2 + m1*r1^2 + m2*(r1^2 + r2^2 + 2*r1*r2*cos(q2));
m12 = J2 + m2*(r2^2 + r1*r2*cos(q2));
m21 = m12;
m22 = J2 + m2*r2^2;
M = [m11, m12; m21, m22];
h = m2*r1*r2*sin(q2);
C_vec = [-h*(2*dq1*dq2 + dq2^2);
          h*dq1^2];
g1 = (m1+m2)*g*r1*cos(q1) + m2*g*r2*cos(q1+q2);
g2 = m2*g*r2*cos(q1+q2);
G_vec = [g1; g2];
ddq = M \ (tau - C_vec - G_vec);
ddq_1 = ddq(1);
ddq_2 = ddq(2);

end
