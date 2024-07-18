clear
close all
clc

%% Observer type

coop = false;
local = not(coop);

% no_observer = true;
no_observer = false;

%% Model

global A B c F L_obs

A = [0 1; 880.87 0];
B = [0; -9.9453];

if no_observer == true
    C = eye(2);
    D = zeros(2,1);
else
    % Observer needed
    C = [708.27 0];
    D = 0;

    L_obs = place(A',C',[-40 -50])';
end

x_0 = [0 0]';
x_0_Leader = [0 2]';

%% Control of the plant

K0 = place(A,B,[2*1i -2*1i]); % sin
% K0 = place(A,B,[0 -5]); % constant 
% K0 = acker(A,B,[0 0]); % ramp
A = A-B*K0;


%% Topology

% Chain topology
% Ad = [0 0 0 0 0 0
%     2 0 0 0 0 0
%     0 6 0 0 0 0
%     0 0 1 0 0 0
%     0 0 0 1 0 0
%     0 0 0 0 3 0];
% 
% D_weight = diag([0 2 6 1 1 3]);
% 
% L = D_weight-Ad;
% 
% G = diag([1 0 0 0 0 0]);

% Star topology
% Ad = zeros(6,6);
% 
% D_weight = zeros(6,6);
% 
% L = D_weight-Ad;
% 
% G = diag([1 2 1 2 2 1]);

% Mixed topology
Ad = zeros(6,6);
Ad(2,1) = 1;
Ad(5,1) = 7;
Ad(3,2) = 4;
Ad(6,2) = 5;
Ad(1,3) = 2;
Ad(6,4) = 2;

D_weight = diag([2 2 4 6 7 7]);

L = D_weight-Ad;

G = diag([0 1 0 6 0 0]);
 
%% Tuning of parameters: Problem of SVFB control

minimum_eig = min(eig(L+G));
c = 1/(2*real(minimum_eig));

q = 1;
Q = q*eye(2);
R = q/10;

P = are(A,B*R^-1*B',Q);
K = (R\B')*P;

%% Tuning of parameters: Cooperative observer

if coop == true
    
    minimum_eig = min(eig(L+G));
    
    q = 10;
    Q = q*eye(2);
    R = q/10;
    
    P = are(A',C'*(R\C),Q);
    F = P*(C'/R);
    
    c = 1/(2*real(minimum_eig));
    
    % return
end
%% Tuning of parameters: Local observer

if local == true
    minimum_eig = min(eig(L+G));
    
    q = 1;
    Q = q*eye(2);
    R = q/10;
    
    P = are(A',C'*(R\C),Q);
    F = - P*C'*R^(-1);
    c = 1/(2*real(minimum_eig));
    
    max(eig(A+c*F*C))
end

%% Simulation

Tsim = 10;

if no_observer == true
    open("Lev_project_control_no_obs.slx")
else
    
    if coop == true
        open("Lev_project_control_coop_obs.slx")
    end
    
    if local == true
        open("Lev_project_control_local_obs.slx")
    end
end
