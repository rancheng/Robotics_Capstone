syms th phi dth dphi u;
params = struct();

params.g = 9.81;
params.mr = 0.25;
params.ir = 0.0001;
params.d = 0.1;
params.r = 0.02;

% 1. Learn how to use the symbolic toolbox in MATLAB
% you will need the state variables, and control input to be declared as symbolic

% 2. call your "eom" function to get \ddot{q} symbolically
qdd = eom(params,th,phi,dth,dphi,u);

% 3. Linearize the system at 0 (as shown in lecture)
% You should end up with A (4x4), and b (4x1)
%qdd = subs(qdd,{th,dth,phi,dphi,u},{0,0,0,0,0})
f = [dth;dphi;qdd];
res = diff(f,th);
A = jacobian(f,[th,phi,dth,dphi])
b = jacobian(f, u)
% th = 0;
% dth = 0;
% phi = 0;
% dphi = 0;
% u = 0;
% A = subs(A,phi,0);
% A = subs(A,th,0);
% A = subs(A,dth,0);
% A = subs(A,dphi,0);
% b = subs(b,u,0);
% S= A(3)
% N = b(3)
A_l = double((subs(A,{th,phi,dth,dphi,u},{0,0,0,0,0})))
% 0.6324    0.0975    0.2785    0.5469
b_l = double((subs(b, {u,phi}, {0,0})))
% N = subs(N,{th,dth,phi,dphi,u},{0,0,0,0,0});

% 4. Check that (A,b) is  controllable
% Number of uncontrollable states should return 0
%A_linearized = [0,0,1,0;0,0,-14715,2452.5;1,0,0,0;0,1,0,0];
% A_linearized = [0,0,1,0;0,0,0,1;0,-14715,0,245;0,-14715,0,245];
% b_linearized = [0;0;-370000;60000];
Co = ctrb(A_l,b_l)
unco = length(A_l) - rank(Co)
k = lqr(A_l,b_l,0.08*eye(4),1000,0)


% 5. Use LQR to get K as shown in the lecture
