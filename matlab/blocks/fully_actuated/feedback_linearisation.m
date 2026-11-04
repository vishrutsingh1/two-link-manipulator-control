function t = linearisation(q, q_dot, ve)
% Feedback linearisation (computed-torque) law: t = M(q)*ve + C(q,q_dot) + G(q)
% Copy of the "Linearisation" MATLAB Function block in
% simulink/fully_actuated_manipulator.slx.

l1 = 1; l2 = 0.8; m1 = 0.5; m2 = 1.5;
j1 = 5; j2 = 5; g = 9.8;

m11 = m1*l1^2 + m2*(l1^2 + l2^2 + 2*l1*l2*cos(q(2))) + j1 + j2;
m12 = m2*(l1*l2*cos(q(2)) + l2^2) + j2;
m21 = m12;
m22 = m2*l2^2 + j2;
M = [m11, m12; m21, m22];

h_val = m2*l1*l2*sin(q(2));
C_vec = [-h_val*q_dot(2)^2 - 2*h_val*q_dot(1)*q_dot(2);
          h_val*q_dot(1)^2];

G1 = (m1 + m2)*l1*g*cos(q(1)) + m2*g*l2*cos(q(1) + q(2));
G2 = m2*g*l2*cos(q(1) + q(2));
G_vec = [G1; G2];

t = M * ve + C_vec + G_vec;

end
