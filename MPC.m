function [steer, y] = fcn(ref, y, ydot, yaw, yawrate, steer)
% MPC parameters
m = 1723.8;
Iz = 4175;
L = 2.7;
lf = 1.24;
lr = L-lf;
Cf = 67248;
Cr = 53248;
Ts = 0.05;

vx = 30*1000/3600;
Tref = 0.3;
alpha = exp(-Ts/Tref);

A = [0 1 0 0;
    0 -2*(Cf+Cr)/m/vx 0 -vx-2*(lf*Cf-lr*Cr)/m/vx;
    0 0 0 1;
    0 -2*(lf*Cf-lr*Cr)/Iz/vx 0 -2*(lf^2*Cf+lr^2*Cr)/Iz/vx];

B = [0;
    2*Cf/m;
    0;
    2*lf*Cf/Iz];
C = [1 0 0 0;
    0 0 1 0];

u_size = 1;
y_size = 2;
x_size = 4;

Ad = eye(x_size) + Ts*A;
Bd = Ts*B;
Cd = C;
Dd = zeros(y_size,u_size);

Q = eye(y_size);
% R = 0.1*eye(u_size);
N1 = 1; % Preferably always 1.
N2 = 5;
Nu = 5;

A = [Ad Bd;
    zeros(u_size,x_size) eye(u_size)];
B = [zeros(x_size,u_size);
    eye(u_size)];
C = [Cd Dd];

F = computeF(A,C,N1,N2);
H = computeH(A,B,C,N1,N2,Nu,y_size,u_size);

Qbar = computeQbar(Q, N1, N2);
% Rbar = computeRbar(R, Nu);
Rbar = 0.5*eye(Nu);
IB = eye(u_size*Nu);
IL = computeIL(u_size, Nu);

P = H'*Qbar*H + Rbar;
P = (P+P')/2;
% AA = [IB; -IB; IL; -IL; H; -H];
AA = [IB; -IB; IL; -IL];

%% States and Inputs

augStates = [y; ydot; yaw; yawrate; steer];

Y = [y; yaw];
% steer
%% MPC with constraints

f = F*augStates;
Ref = computeRef(ref(:, 1:1+N2), Y, y_size, N1, N2, alpha);

b = computeb(f,steer, Ts, Nu, u_size); % Change input constraints inside the function
q = -H'*Qbar*(Ref-f);

Du = quadprog(P,q,AA, b,[],[],[],[],zeros(Nu,1), optimoptions('quadprog','Algorithm','active-set'));


du = Du(u_size,1);
augStates = A*augStates + B*du;
Y = F*augStates + H*Du;

y = Y(1,1);
steer = augStates(5,1);


end

%% Functions

function Ref = computeRef(true_ref, Y, y_size, N1, N2, alpha)
Ref = zeros(y_size * (N2-N1+1), 1);
for i=N1:N2
    Ref(y_size*i-1:y_size*i,1) = true_ref(:,i+1) - alpha^(i+1)*(true_ref(:,1) - Y);
%     Ref(y_size*i-1:y_size*i,1) = true_ref(:,i+1); % No alpha
end
end

function Qbar = computeQbar(Q, N1, N2)
Q_size = size(Q,1);
Qbar = zeros(Q_size*N2);
for i = N1:N2
    for j = N1:N2
        if i == j
            Qbar(Q_size*i-1:Q_size*i,Q_size*j-1:Q_size*j) = Q;
        else
            Qbar(Q_size*i-1:Q_size*i,Q_size*j-1:Q_size*j) = zeros(Q_size);
        end
    end
end

end

% function Rbar = computeRbar(R, Nu)
% R_size = size(R,1);
% Rbar = zeros(R_size*Nu,R_size*Nu);
% for i = 1:Nu
%     for j = 1:Nu
%         if i == j
%             Rbar(i:R_size*i,R_size*j-1:R_size*j) = R;
%         else
%             Rbar(R_size*i-1:R_size*i,R_size*j-1:R_size*j) = zeros(R_size);
%         end
%     end
% end
% end

function IL = computeIL(u_size, Nu)
IL = zeros(Nu);
for i = 1:Nu
    for j = 1:Nu
        if i < j
            IL(i,j) = zeros(u_size);
        else
            IL(i,j) = eye(u_size);
        end
    end
end
end


function b = computeb(f, u, Ts, Nu, u_size)
dumax = 4*pi*Ts;
dumin = -4*pi*Ts;
umax = pi;
umin = -pi;
ymax = [1;
        1];
ymin = [-1;
        -1];

b1 = zeros(Nu,1); b2 = zeros(Nu,1);
b3 = zeros(Nu,1); b4 = zeros(Nu,1);
b5 = zeros(Nu,1); b6 = zeros(Nu,1);

for i = 1:Nu
    b1(i) = dumax;
    b2(i) = -dumin;
    b3(i) = umax - u;
    b4(i) = -(umin - u);
%     b5(u_size*i-1:u_size*i,1) = ymax - f(u_size*i-1:u_size*i,1);
%     b6(u_size*i-1:u_size*i,1) = -(ymin - f(u_size*i-1:u_size*i,1));
end

% b = [b1;b2;b3;b4;b5;b6];
b = [b1;b2;b3;b4];
end

function F = computeF(A,C,N1,N2)
C_row = size(C,1);
C_column = size(C,2);
F = zeros(C_row*N2,C_column);
for i=N1:N2
    F(C_row*i-1:C_row*i,1:C_column) = C*(A^i);
end

end

function H = computeH(A,B,C,N1,N2,Nu, y_size, u_size)
H = zeros(y_size*N2, u_size*Nu);
for j=N1:N2
    for i=1:Nu
        if j<i
            H(y_size*j-1:y_size*j,i:Nu)=zeros(y_size,u_size);
        else
            H(y_size*j-1:y_size*j,i:Nu) = C*A^(j-i)*B;
        end
    end
end
end
