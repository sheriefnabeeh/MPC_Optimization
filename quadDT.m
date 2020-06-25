function [xk1] = quadDT(xk, uk,Ts)

%discretizing the model using first order runga kutta for approximation 
% sampling time is set to 0.01 as a first trial . 
Ts= 0.09;
M = 1;
delta = Ts/M;
xk1 = xk;
u=uk;
for ct=1:M
    
    xk1 = xk1 + delta*quad_nonlin(xk1,u);

end
