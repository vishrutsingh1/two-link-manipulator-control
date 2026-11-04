function v = fcn(q, q_dot)
% State-feedback part of the outer loop (virtual input v).
% Copy of the "transformation" MATLAB Function block in
% simulink/fully_actuated_manipulator.slx.
% Gains from pole placement at {-10, -11, -12}:
%   (s+10)(s+11)(s+12) = s^3 + 33 s^2 + 362 s + 1320
% The integral gain Ki = 1320 is implemented with Simulink gain/integrator blocks.

kp = 362;
kd = 33;
z1 = q;
z2 = q_dot;
v_pos = kp * z1;
v_vel = kd * z2;
v = v_pos + v_vel;

end
