%quadcopter continuous model with 12 states and deriving the jacobian for 
%implementing the first degree runga kutta

% states : 
function [dxdt] = quad_nonlin(X, u)
m = 0.65;
g =9.8; 
Ixx = 7.5e-3; 
Iyy= 7.5e-3;
Izz= 1.3e-2;

%syms x x_dot y y_dot z z_dot phi phi_dot theta theta_dot yaw yaw_dot ;
%needed z psi x phi y theta

%syms u1 u2 u3 u4 ;
x= X(5)  ; 
x_dot=X(6) ; 
y=X(9) ; 
y_dot=X(10) ; 
z=X(1); 
z_dot=X(2) ; 
phi=X(7); 
phi_dot=X(8) ; 
theta=X(11) ; 
theta_dot=X(12) ; 
yaw =X(3) ; 
yaw_dot=X(4) ; 

u1= u(1);
u2=u(2);
u3=u(3);
u4=u(4); 

%state space representation , state derivatives
dxdt=X;
dxdt(5) = x_dot; 
dxdt(6) = u1*((sin(phi)*sin(yaw)+ cos(phi)*sin(theta)*cos(yaw)));
dxdt(9) = y_dot ; 
dxdt(10) =u1*((-sin(phi)*cos(yaw))+ cos(phi)*sin(theta)*sin(yaw));
dxdt(1) = z_dot ; 
dxdt(2) = (u1/m)*(cos(phi)*cos(theta))- g; 
dxdt(7) = phi_dot ; 
dxdt(8) = u2/Ixx + theta_dot*yaw_dot - theta_dot*yaw_dot*(Izz/Ixx);
dxdt(11) = theta_dot ; 
dxdt(12)= u3/Iyy - phi_dot*yaw_dot + phi_dot*yaw_dot*(Izz/Iyy);
dxdt(3)= yaw_dot ;
dxdt(4)=u4/Izz + phi_dot*theta_dot*(Ixx/Izz)+phi_dot*theta_dot*(Iyy/Izz); 

%diff of each state function relative to each state .
% dx2phi = diff(dxdt(2) ,phi);
% dx2theta= diff(dxdt(2) ,theta);
% dx2yaw= diff(dxdt(2) ,yaw);
% 
% dx4phi = diff(dxdt(4) ,phi);
% dx4theta= diff(dxdt(4) ,theta);
% dx4yaw= diff(dxdt(4) ,yaw);
% 
% dx6phi = diff(dxdt(6) ,phi);
% dx6theta= diff(dxdt(6) ,theta);
% dx6yaw= diff(dxdt(6) ,yaw);
% 
% dx8phi_dot = diff(dxdt(8) ,phi_dot);
% dx8theta_dot= diff(dxdt(8) ,theta_dot);
% dx8yaw_dot= diff(dxdt(8) ,yaw_dot);
% 
% dx10phi_dot = diff(dxdt(10) ,phi_dot);
% dx10theta_dot= diff(dxdt(10) ,theta_dot);
% dx10yaw_dot= diff(dxdt(10) ,yaw_dot);
% 
% dx12phi_dot = diff(dxdt(12) ,phi_dot);
% dx12theta_dot= diff(dxdt(12) ,theta_dot);
%   
