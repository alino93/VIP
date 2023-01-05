function [x_hat, L] = HGObserver(a_body, W, th, Fs)
%% Observer gain
la= 3.33;

A = [0 1 0 0 0 0;0 0 0 0 0 0;0 0 0 1 0 0;0 0 0 0 0 0;0 0 0 0 0 1;0 0 0 0 0 0;];
B = [0 0 0;1 0 0;0 0 0;0 1 0;0 0 0;0 0 1];
C = [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0];
I = eye(6);
nsys = 6;
nout = 3;
% Define the decision variables P and R
P = sdpvar(nsys, nsys);
R = sdpvar(nsys, nout);

% Set LMI constraints
lmi = A'*P + P*A - C'*R'- R * C + la * I;

% Construct LMIs for solver
F = [P >= 0];
F = F + [lmi <= 0];

% Choose solver
ops = sdpsettings('solver','sedumi'); % need to install
ops.verbose = 0;
% Solve the LMI
diagnostics = optimize(F, [ ], ops);
% optimize(Constraints, Objective, options)

%Check the results of optimization
if diagnostics.problem == 0
    disp('Feasible from Solver')
elseif diagnostics.problem == 1
    disp('lnfeasible from Solver')
else
    disp('Something else happened')
end
    
% Display the results
P = double(P);
R = double(R);
K = P\R;
L = diag([th,th^2,th,th^2,th,th^2]) * K;

%% Nonlinear observer quat estimation
T = 1/Fs;
g = 9.81;
ge = norm(a_body(:,1));
a_0 = a_body(:,1)/ge ;

%time
t = 0:T:T*(length(W(1,:))-1);

%% Observer
% initialize
x_hat = zeros(nout,length(t));
z_hat = zeros(nsys,length(t));

x_hat(:,1) = [atan2(a_0(2),a_0(3));-asin(a_0(1));0.01];

x_h = x_hat(:,1);
x_dot = [1 sin(x_h(1))*tan(x_h(2)) cos((x_h(1)))*tan(x_h(2));
         0 cos(x_h(1)) -sin(x_h(1));
         0 sin(x_h(1))/cos(x_h(2)) cos(x_h(1))/cos(x_h(2))] * W(:,1);

z_hat(:,1) = [x_h(1);x_dot(1);x_h(2);x_dot(2);x_h(3);x_dot(3)];

for i=2:1:length(t)
    ge = norm(a_body(:,i));
    a_0 = a_body(:,i)/ge;
    
    w = W(:,i-1);
    x_h = x_hat(:,i-1);
    x_dot = [1 sin(x_h(1))*tan(x_h(2)) cos((x_h(1)))*tan(x_h(2));
             0 cos(x_h(1)) -sin(x_h(1));
             0 sin(x_h(1))/cos(x_h(2)) cos(x_h(1))/cos(x_h(2))] * w;
    
    f_1 = x_dot(1) * tan(x_h(2)) * (w(2)*cos(x_h(1)) - w(3)*sin(x_h(1))) + ...
        x_dot(2)/cos(x_h(2))^2 * (w(3) * cos(x_h(1)) + w(2) * sin(x_h(1))); 
    f_2 = -x_dot(1) * (w(3)*cos(x_h(1)) + w(2)*sin(x_h(1)));
    f_3 = x_dot(1) * (w(2)*cos(x_h(1)) - w(3)*sin(x_h(1))) / cos(x_h(2)) + ...
        x_dot(2) * tan(x_h(2)) * (w(3) * cos(x_h(1)) + w(2) * sin(x_h(1)))/cos(x_h(2));
   
    f_hat = [f_1;f_2;f_3];

    y = [1*atan2(a_0(2),a_0(3));-asin(a_0(1));z_hat(5,i-1)];
    
    z_hat(:,i) = z_hat(:,i-1) + T * (A * z_hat(:,i-1) + ...
    B * f_hat + L * (y - C * z_hat(:,i-1)));
    
    % suppress results outside -pi,pi
    if z_hat(1,i)> pi
        z_hat(1,i) = pi;
    elseif z_hat(1,i)<-pi
        z_hat(1,i) = -pi;
    end
    x_hat(1,i) = z_hat(1,i);
    x_hat(2,i) = z_hat(3,i);
    x_hat(3,i) = z_hat(5,i);
        
%     if x_dot(3)<0.4
%         x_dot(3) = 0; 
%     end
     x_hat(3,i) = x_hat(3,i-1) + T * (x_dot(3));
       
end
end
