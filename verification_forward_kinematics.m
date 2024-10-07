% Define the DH parameters for UR10 with offsets for theta2 and theta4
a = [0, 0.6127, 0.5716, 0, 0, 0];
alpha = [-pi/2, 0, 0, -pi/2, pi/2, 0];
d = [0.128, 0, 0, 0.1639, 0.1157, 0.0922];
theta = [0, 0, 0, 0, 0, 0]; % Assuming initial joint angles are all zero
offsets = [0, -pi/2, 0, -pi/2, 0, 0]; % Offsets for theta2 and theta4

% Define the robot using the SerialLink function with offsets
ur10 = SerialLink([Revolute('a', a(1), 'alpha', alpha(1), 'd', d(1), 'offset', offsets(1)), ...
                   Revolute('a', a(2), 'alpha', alpha(2), 'd', d(2), 'offset', offsets(2)), ...
                   Revolute('a', a(3), 'alpha', alpha(3), 'd', d(3), 'offset', offsets(3)), ...
                   Revolute('a', a(4), 'alpha', alpha(4), 'd', d(4), 'offset', offsets(4)), ...
                   Revolute('a', a(5), 'alpha', alpha(5), 'd', d(5), 'offset', offsets(5)), ...
                   Revolute('a', a(6), 'alpha', alpha(6), 'd', d(6), 'offset', offsets(6))], ...
                   'name', 'UR10');

% Plot the robot
ur10.plot(theta);
ur10.teach

% Display the end-effector orientation
T = ur10.fkine(theta);
disp('End-effector orientation:');
disp(T.t);
    