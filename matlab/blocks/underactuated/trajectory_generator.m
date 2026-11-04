function [q_ref1, q_ref2] = trajectory_generator(time)
% Constant set-point at the chosen equilibrium (q1 + q2 = pi/2).
% Copy of the MATLAB Function block in simulink/underactuated_manipulator.slx.

q_ref1 = 1.0;
q_ref2 = 0.5708;   % pi/2 - 1.0

end
