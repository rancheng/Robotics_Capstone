
function xhat = EKFstudent(t, z)
  % In this exercise, you will batch-process this data: you are provided a vector of timestamps (of length T), and a 3xT matrix of observations, z.
  xhat = zeros(2,length(t));

  % Student completes this
  P = zeros(2,2,length(t));
  Q = [0.1 0;0 0.1];
  R = [0.009 0 0;0 0.009 0;0 0 0.0076];
  g = 1;
  xhat(:,1) = [2,0];
  P(:,:,1) = [5 5;5 5];
  %xhat(:,1) = [5;3];
  h = [g*sind(xhat(1,1));g*cosd(xhat(1,1));xhat(2,1)];
  xkp_o = xhat(:,1);
  
  %H = [9.81*(pi/180)*cosd(xhat(1)) 0;-9.81*(pi/180)*sind(xhat(1)) 0;1 0];
 % h = [9.81*sind(xhat(1));9.81*cosd(xhat(1));xhat(2)];
%  xhat(:,1) = [5;3];
 % P(:,:,1) = [0.5 0.5;0.5 0.5];
  for i=2:length(t)
      dt = t(i) - t(i-1);
      A = [1 dt;0 1];
      xkp = A*xhat(:,i-1);
      Pkp = A*P(:,:,i-1)*A' + Q;
      H = [g*cosd(xhat(1,i))*(pi/180) 0; -g*sind(xhat(1,i))*(pi/180) 0;0 1];
      h = [g*sind(xkp(1)); g*cosd(xkp(1)); xkp(2)];
      K = Pkp*H'/(H*Pkp*H' + R);
      xhat(:,i) = xkp + K*(z(:,i) - h);
      P(:,:,i) = (eye(2) - K*H)*Pkp;
      %h = h + H*(xkp - xkp_o);
     % xkp_o = xkp;
  end
end
