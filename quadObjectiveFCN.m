function J= quadObjectiveFCN(xref,u,x,Ts,u0)

N=10;
% xref =([5;0;0;0;-5;0;0;0;5;0;0;0]);
Q = diag([1,1,1,1,2,0.25,1,1,2,0.25,1,1]);

R =  diag([0.01,0.01,0.01,0.01]);

%% Cost Calculation
% Set initial plant states, controller output and cost.
xk = x;
uk = u(:,1);
J = 0;
% Loop through each prediction step.
for ct=1:N
    % Obtain plant state at next prediction step.
    xk1 = quadDT(xk, uk, Ts);
    % accumulate state tracking cost from x(k+1) to x(k+N).
    J = J + (xk1-xref)'*Q*(xk1-xref);
    % accumulate MV rate of change cost from u(k) to u(k+N-1).
    if ct==1
        J = J + (uk-u0)'*R*(uk-u0);
    else
        J = J + (uk-u(:,ct-1))'*R*(uk-u(:,ct-1));
    end
    % Update xk and uk for the next prediction step.
    xk = xk1;
    if ct<N
        uk = u(:,ct+1);
    end
end


