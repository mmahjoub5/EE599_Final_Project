function [c,ceq] = dynConst(z,xinit,dt)
% non linear dynamic constraints
    
    % c non linear inequality contraints
    c = [];
    N = length(z);
    H = (N-4)/6;
    ceq = zeros(4*H,1);
    % there will be 3*H equality constraints
    for iter = 1:H
        ceq(iter) = z(iter+1)-z(iter)- dt * z(3*(H+1)+iter) *cos(z(2*(H+1)+iter));              % constraint in x
        ceq(H+iter) = z((H+1)+iter+1)-z(H+1+iter)-dt* z(3*(H+1)+iter) *  sin(z(2*(H+1)+iter));  % constraint in y
        ceq(2*H+iter) = z(2*(H+1)+iter+1)-z(2*(H+1)+iter)-dt*z(4*(H+1)+iter);                   % constraint in theta
        ceq(3*H+iter) = z(3*(H+1)+iter+1) - z(3*(H+1)+iter) - dt * z(5*(H+1)+iter - 1);         % constrain in v
    end
    % initial state constraints
    ceq = [ceq;z(1)-xinit(1);z(H+2)-xinit(2);z(2*H+3)-xinit(3); z(3*H+4)-xinit(4)];
%     innerLoop = [norm(z(1:H+1)) + norm(z(H+1+1:2*H+2)) - 1];
%     outerLoop = [ 5 - norm(z(1:H+1)) + norm(z(H+1+1:2*H+2))];
%     c= [innerLoop; outerLoop];
end