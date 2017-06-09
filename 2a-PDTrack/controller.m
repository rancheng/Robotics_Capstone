
function u = controller(params, t, x, xd)
  % x = current position
  % xd = current velocity

  % Use params.traj(t) to get the reference trajectory
  % e.g. (x - params.traj(t)) represents the instaneous trajectory error

  % params can be initialized in the initParams function, which is called before the simulation starts
  
  % SOLUTION GOES HERE -------------
  kp = -3000;
  kv = -100;
  lumda = -0.1;
  % ki = 10;
  errT = x - params.traj(t);
  x0 = params.traj(0);
  % vt = x0*exp(lumda*t);
  u = kv*(xd)^3 + kp * errT;
end