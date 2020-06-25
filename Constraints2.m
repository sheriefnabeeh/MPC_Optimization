function [c,ceq] = Constraints2(u,x,Ts,N)

zMin = -100;
zMax = 100;

c = zeros(N*2,1);

xk = x;
uk =u(:,1);

for ct=1:N
    % obtain new cart position at next prediction step
    xk1 = quadDT(xk, uk, Ts);
    % -z + zMin < 1
    c(2*ct-1) = -xk1(1)+zMin;
    % z - zMax < 0
    c(2*ct) = xk1(1)-zMax;
    % update plant state and input for next step
    xk = xk1;
    if ct<N
       uk = u(:,ct+1);
    end
end
ceq=[];

