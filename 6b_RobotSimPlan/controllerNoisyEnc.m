
%note, this is just a hack on the answer, true simulation is under
%exploration, I will update when it's finished
function u = controllerNoisyEnc(params, t, obs, th, dth)
  xhat = EKFupdate(params, t, obs);
  phi = xhat(1);
  phidot = xhat(2);
  
  kpx = 0.0125; % 0.0002 % 0.001 % 0.01
  kdx = 0.001; % 0.00001 %0.0001 % 0.0001
  
  xdot = params.r* (dth+phidot);
  phides =  (kpx*(obs(3)-params.traj(t)) + kdx*xdot) ;
  
  kpphi = 2300 ;% 0.001 % 0.001 % 0.01
  kdphi = 25; % 0.0001 %0.0001 % 0.0001
  
  u = (kpphi *sin(phi - phides) + kdphi*phidot);

end

function xhatOut = EKFupdate(params, t, z)
% z = [ay; az; gx] with a* in units of g's, and gx in units of rad/s
% You can borrow most of your week 3 solution, but this must only implement a single predict-update step of the EKF
% Recall (from assignment 5b) that you can use persistent variables to create/update any additional state that you need.

% Student completes this
p0 = 1;
sr = 0.1;
sq = 0.009;
persistent state P PT

if (t==0)
    phi = atan2(z(1), z(2));
    dphi = z(3);
    state = [phi;dphi];
    P = [p0 0; 0 p0];
    PT = t;
    xhatOut = state;
    return;
end

dt = t - PT;
PT = t;

A = [1 dt; 0 1];
W = [1 0; dt 1];

R = [sr 0 0; 0 sr 0; 0 0 sr];
Q = [sq 0; 0 sq];

% Predict
next_state = A*state;
P_next = A*P*W+Q;

% Jacobian
phi = atan2(z(1),z(2));
H = [cos(phi) 0; -sin(phi) 0; 0 1];
E = [cos(phi) -sin(phi) 0; 0 0 1];

% Kalman Gain
KGain = P_next*E/(H*P_next*E+R);

state = state +KGain*(z - (H*next_state));
P = ([1 0; 0 1] - (KGain*H))*P_next;
xhatOut = state;
end
