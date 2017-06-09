function qdd = eom(params, th, phi, dth, dphi, u)
  % This is the starter file for the week5 assignment

  % Provided params are
  % params.g: gravitational constant
  % params.mr: mass of the "rod"
  % params.ir: rotational inertia of the rod
  % params.d: distance of rod CoM from the wheel axis
  % params.r: wheel radius


  A = (params.mr*params.r*params.d*cos(phi));
  B = (params.ir+params.mr*(params.d)^2);
  C = (params.mr*params.g*params.d);
  D = (params.mr*(params.r)^2);
  E = (params.r*params.mr*params.d*cos(phi));
  F = (params.mr*params.r*params.d);
  tau = u;
  phidd = ((A*F*sin(phi)*dphi^2-C*D*sin(phi)+A*tau+D*tau)/(A*E-B*D));
  thdd = (-(A*phidd+B*phidd-C*sin(phi)+tau)/A);
  qdd = [thdd;phidd];
end