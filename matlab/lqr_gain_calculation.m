% LQR gain for the underactuated two-link manipulator (joint 2 has no motor).
%
% Steps:
%   1. Partial feedback linearisation: choose v = q2_ddot as the new input,
%      and express q1_ddot from the (unactuated) joint-2 equation of motion.
%   2. Linearise the resulting system about the equilibrium
%      q1 = 1.0 rad, q2 = pi/2 - 1.0 rad (link 2 horizontal, so G2 = 0).
%   3. Check controllability and compute the LQR gain K.
%
% Requires: Symbolic Math Toolbox, Control System Toolbox.

syms q1 q2 dq1 dq2 v_input real;

% Manipulator parameters (from the assessment brief)
r1 = 1.0; r2 = 0.8;     % link lengths [m]
m1 = 0.5; m2 = 1.5;     % link masses [kg]
J1 = 5.0; J2 = 5.0;     % link inertias [kg m^2]
g  = 9.8;               % gravity [m/s^2]

% Inertia matrix terms, Coriolis and gravity at joint 2
m11 = J1 + J2 + m1*r1^2 + m2*(r1^2 + r2^2 + 2*r1*r2*cos(q2));
m12 = J2 + m2*(r2^2 + r1*r2*cos(q2));
m21 = m12;
m22 = J2 + m2*r2^2;
h  = m2*r1*r2*sin(q2);
c2 = h*dq1^2;
g2 = m2*g*r2*cos(q1+q2);

% Equilibrium point: q1 + q2 = pi/2
q1e = 1.0;
q2e = pi/2 - 1.0;

% Joint-2 equation solved for q1_ddot, with q2_ddot = v
ddq1_expr = (-c2 - g2 - m22*v_input) / m21;
f_pfl = [dq1; dq2; ddq1_expr; v_input];

% Linearise about the equilibrium
A_sym = jacobian(f_pfl, [q1, q2, dq1, dq2]);
B_sym = jacobian(f_pfl, v_input);

A_val = double(subs(A_sym, {q1,q2,dq1,dq2,v_input}, {q1e,q2e,0,0,0}));
B_val = double(subs(B_sym, {q1,q2,dq1,dq2,v_input}, {q1e,q2e,0,0,0}));

disp('A ='); disp(A_val);
disp('B ='); disp(B_val);
disp(['Controllability rank = ', num2str(rank(ctrb(A_val, B_val)))]);

% LQR weights: penalise the unactuated joint q2 most
%Q = diag([50 200 5 20]);   % earlier, more aggressive tuning
Q = diag([10 100 1 10]);
R = 1;
[K, ~, e] = lqr(A_val, B_val, Q, R);

disp('K ='); disp(K);          % used in the "transformation" block of underactuated_manipulator.slx
disp('Closed-loop poles ='); disp(e);
