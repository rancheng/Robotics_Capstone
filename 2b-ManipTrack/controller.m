
function u = controller(params, t, X)
  u=[0; 0];
  % 1. write out the forward kinematics, such that p = FK(theta1, theta2)
  % 2. Let e = p - params.traj(t) be the task-space error
  % 3. Calculate the manipulator Jacobian J = d p / d theta
  % 4. Use a "natural motion" PD controller, u = - kp * J^T * e - kd * [dth1; dth2]
  l = 0.5;
  kp = 30000;
  kd = 100;
  p = l*[cos(X(1)) + cos(X(1) + X(2)), sin(X(1)) + sin(X(1) + X(2))]';
  % theta = [X(1),X(2)];
  e = p - params.traj(t);
  J = l*[-sin(X(1)) - sin(X(1) + X(2)), -sin(X(1) + X(2)); cos(X(1)) + cos(X(1) + X(2)), cos(X(1) + X(2))];
  u = -1 * kp * J' * e - kd * [X(3); X(4)];
  
end

