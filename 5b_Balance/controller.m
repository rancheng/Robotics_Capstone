
function u = controller(params, t, phi, phidot)
  % STUDENT FILLS THIS OUT
  % 
  % Initialize any added state like this:
  %t
  persistent newstate t_last
  if isempty(newstate)
    % initialize
    newstate = 0;
    t_last = 0;
  end
  dt = t_last - t;
  newstate = newstate + dt*phi;
  t_last = t;
  kp = 53;
  kd = 2.9;
  ki = -2000;
  
  u = kp*phi + kd*phidot + ki*newstate;

end

