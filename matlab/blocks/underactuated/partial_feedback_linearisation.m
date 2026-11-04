function tau = linearisation(q, dq, v)
% Partial feedback linearisation (collocated on the unactuated joint):
%   tau1 = alpha(x) * v + beta(x),  tau2 = 0
% so that q2_ddot = v. Copy of the "Linearisation" MATLAB Function block in
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
h = m2*r1*r2*sin(q2);
c1 = -h*(2*dq1*dq2 + dq2^2);
c2 = h*dq1^2;
g1 = (m1+m2)*g*r1*cos(q1) + m2*g*r2*cos(q1+q2);
g2 = m2*g*r2*cos(q1+q2);
gain_alpha = (m12 - (m11*m22)/m21);
drift_beta = (c1 + g1) - (m11/m21)*(c2 + g2);
tau1 = gain_alpha * v + drift_beta;
tau = [tau1; 0];

end
