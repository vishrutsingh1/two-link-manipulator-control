function [q_ref1, q_ref2] = trajectory_generator(time)
% Reference trajectories for the fully actuated case.
% Copy of the MATLAB Function block in simulink/fully_actuated_manipulator.slx.
% Both converge to 1.25 rad.

q_ref1 = 1.25 - (7/5)*exp(-time) + (7/20)*exp(-4*time);
q_ref2 = 1.25 + exp(-time) - (1/4)*exp(-4*time);

end
