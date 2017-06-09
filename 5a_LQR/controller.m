
function u = controller(params, t, X)
  % You have full state feedback available

  % After doing the steps in simLinearization, you should be able to substitute the linear controller u = -K*X
%   qdd = eom(params,th,phi,dth,dphi,u);
%   f = [dth;dphi;qdd];
%   res = diff(f,th);
%   A = jacobian(f,[phi,dth,dphi,u]);
%   b = jacobian(f, u);
%   A_l = double(vpa(subs(A,{th,phi,dth,dphi,u},{X(1),X(2),X(3),X(4),0})));
%   b_l = double(vpa(subs(b,{u,phi},{0,X(2)})));
%   Co = ctrb(A_l,b_l);
%   unco = length(A_l) - rank(Co);
%   k = lqr(A_l,b_l,10000*eye(4),1000,0);
%   24.0375    4.1687    0.5292    2.0488
  k = [-0.0089,   -1.0553,   -0.0112,   -0.1250];
  u = -k*X;
  %u = [0.3716,0.3163,-0.2893,0.4264]*X;
  %0.3716    0.3163   -0.2893    0.4264
end

