clear all
close all
clc;

Scenario = 2;

%System parameter for y'' = R1*y + R2 y' + S*eta + Gam*u, eta' = Q*eta + P y
% -> y'' = f(y,y',eta) + Gam*u,
% R1 = -1143/781;
% R2 = 8/9;
% S = [4/9 -555/1766]
% Gam = 1/9;
% Q = [0 1;-4 -2];
% P = sqrt(2)* [1 ; 0];
syms m1 m2 d k 
thet = pi/4;
mu = m2*(m1 + m2*sin(thet)^2);
mu1 =m1/mu;
mu2 = m2/mu;
A = [0 1 0 0
    0 0 mu2*k*cos(thet) mu2*d*cos(thet)
    0 0 0 1
    0 0 -(mu1+mu2)*k -(mu1+mu2)*d];

B = [0
    mu2
    0
    -mu2*cos(thet)];

C = [1 0 cos(thet) 0];

Gam = C*A*B;
Gam_i = 1/Gam;

cC = [C
      C*A];
cB = [B A*B];
CB_i = inv(cC*cB);

V = null(cC);
V_i = inv(V'*V) ;

N = V_i * V' *(eye(4) - cB*CB_i*cC);

U = [cC; N];
U_i = U\eye(4);

RS = C*A^2*U_i ;

R1 = RS(1,1) ;
R2 = RS(1,2) ;
S = RS(1,3:end) ;

P = N*A^2*B * Gam_i ;

Q = N*A*V ;

VAL = [1 2 1 1]; % m1 m2 k d

Gam_num = subs(Gam,[m1 m2 k d],VAL) ;
R1_num = subs(R1,[m1 m2 k d],VAL)  ;
R2_num = subs(R2,[m1 m2 k d],VAL)  ;
S_num = subs(S,[m1 m2 k d],VAL)  ;

Q_num = subs(Q,[m1 m2 k d],VAL) ;
P_num = subs(P,[m1 m2 k d],VAL) ;

R1 = R1_num ;
R2 = R2_num ;
S = S_num ;
Gam = Gam_num ;
Q = Q_num ;
P = P_num ;

R1 = double(R1);
R2 = double(R2);
S = double(S);
Gam = double(Gam);
Q = double(Q);
P = double(P);

clear mu mu1

K_Q = lyap(Q',eye(2));
K_Q_ = K_Q \ eye(2);
K = norm(K_Q);
K_ = norm(K_Q_);
M = sqrt(K_* K);
mu = 1/(2*K);

t_change = 0.25;


% Funnel function metrics phi = 1/(a*exp(-bt) + c); psi = a*exp(-bt) + c
a_psi = 0;
b_psi = 1;
c_psi = 0.15;

psi_min = c_psi;
sup_psi = a_psi + c_psi;

sup_phi = 1/c_psi; 
inf_phi = 1/(a_psi + c_psi) ;
sup_dphi_phi = a_psi * b_psi / (a_psi + c_psi);


% Initial values
y0 = -0.0925;
dy0 = dRef(0,t_change);
eta0 = [0;0];


phi0 = Phi(0,a_psi,b_psi,c_psi) ;

e0 = y0 - Ref(0,t_change);
e10 = Ek(phi0,e0,0);
de0 = dy0 - dRef(0,t_change);
e20 = Ek(phi0,de0,e10)


% Checking initial conditions
if norm(e10) > 1 
    error('e1 too large')
elseif norm(e20) >1
    error('e2 too large')
end

% Reference metrics
ref_max = 0.4*1;
dref_max = 0.4/2*pi;
ddref_max = 0.4/4*pi^2;

% Epsilons and mus for|e_k| < ve_k
aux_ve1 = sup_dphi_phi + 1;
hat_ve1 = sqrt((1/(2*aux_ve1))^2 + 1/aux_ve1) - 1/2*1/aux_ve1 ;
ve1 = max(norm(e10),hat_ve1);

mu1 = sup_dphi_phi + 1 + Gain(ve1^2)* ve1 ;
gam1 = 2*dGain(ve1^2)*ve1^2 * mu1 + Gain(ve1^2) * mu1;

aux_ve2 = sup_dphi_phi* ( 1 + Gain(ve1^2)* ve1) + gam1 ;
hat_ve2 = sqrt((1/(2*aux_ve2))^2 + 1/aux_ve2) - 1/2*1/aux_ve2 ;

ve2 = max(norm(e20),hat_ve2);

sup_psi = a_psi + c_psi;

% Maximal y and dy
y_max = sup_psi + ref_max ;
dy_max = (ve2 + Gain(ve1^2) * ve1)/inf_phi + dref_max ;

% Maximal internal dynamics
eta_max = M * norm(eta0) + M/mu * norm(P)* y_max ;


% System parameter metrics (eta(0) = 0), |y| < psi.
fmax = norm(R1,2)*y_max + norm(R2,2)*dy_max + norm(S,2)*eta_max; % maximum of f(y,eta)
gmin = norm(Gam,2); % minimal value of g(y,eta) > 0
gmax = gmin;


fmax = round(fmax,1); % Damit man es im paper als fmax <= ... schreiben kann


% Control parameters
kap0 = sup_dphi_phi* ( 1 + Gain(ve1^2)* ve1) + sup_phi*(fmax + ddref_max) + gam1 ;%5
bet = 2*kap0 / (gmin * inf_phi )  % bet = 6 funktioniert mit lambda = 0.5

lambda =  0.75 ;

aa = sup_phi * gmax * bet/lambda;

kap1 = kap0 +  aa; %3

% Setting the threshold when the control becomes active

% Sampling parameters
tau_1 = kap0/kap1^2 ;
tau_2 = (1-lambda)/kap0 ;

%% egal welches u, wenn e_r < lambda
%u_fac = 50/100 ;
%tau_2 = (1-lambda)/(kap0 + sup_phi * gmax * u_fac * bet/lambda) ;

tau = min(tau_1, tau_2) ;% maximale sampling time -> 1/tau minimal sampling rate

if 2 == Scenario
    bet = 4;
    tau = 0.02;
end

if tau < 1e-5
    error('tau too small')
end


%% Simulations
tmax = 1; % Interval of simulation

iter = floor(tmax/tau); % iterations due to tau
t=linspace(0,tmax,iter); % uniform sampling

ref_iter = Ref(t,t_change); % reference at sampling time
dref_iter = dRef(t,t_change); % reference at sampling time

phi_iter = Phi(t,a_psi,b_psi,c_psi);

X0 = [y0,dy0,eta0']; % initial contidions
e00 = X0(1) - ref_iter(1,:); % initial error

%initialization
x_sampled = [X0; zeros(iter,4)];
u_samp = [];

z_con = []; % z_con is the "continuous" solution of the system's dynamics
t_con = []; % t_con is the "continuous" time
e1_iter=[];%e10;
e2_iter=[];%e20;
Error_variables = [];

for k=1:iter-1
    x0 = x_sampled(k,:); % initial state at the beginning t_k
    y = x_sampled(k,1); % initial output value
    dy = x_sampled(k,2); % initial output value
    e = y - ref_iter(k)'; % value e(t_k)
    de = dy - dref_iter(k)'; % value e(t_k)
    e1 = phi_iter(k) * e ;
    e2 = phi_iter(k)*de + Gain(norm(e1)^2)*e1 ;
    if norm(e1) > 1
        error('e1 exceeds 1')
    elseif norm(e2) > 1
        error('e2 exceeds 1')
    end
    if norm(e2) < lambda % control will be active of not
        u_samp(k,:) = 0 ;
    else
        u_samp(k,:) = -bet * e2/(norm(e2)^2); 
    end
    options = odeset(AbsTol=1e-6,RelTol=1e-6);
    [r,z] = ode15s(@(r,z) sampled_ode(r,z,R1,R2,S,Gam,Q,P,u_samp(k),a_psi,b_psi,c_psi,t_change),[t(k),t(k+1)],x0,options);  % integrating the dynamics
    x_sampled(k+1,:) = z(size(z,1),:); % saving the last state in [t_k, t_k+1]
    t_con = [t_con; r]; % stacking the cont. time
    z_con = [z_con; z]; % stacking the cont. states
    e1_iter = [e1_iter, e1];
    e2_iter = [e2_iter, e2];
    %
    % Extracting the e1 and e2from the solution of the ode
    [~,E] = cellfun(@(r,z) sampled_ode(r,z',R1,R2,S,Gam,Q,P,u_samp(k),a_psi,b_psi,c_psi,t_change), num2cell(r), num2cell(z,2),'uni',0);
    E=E';
    E = cell2mat(E);
    Error_variables = [Error_variables; E'];
end

 E = Error_variables;

e1_iter = [e1_iter, E(end)]; %adding the last value

psi = Psi(t_con,a_psi,b_psi,c_psi); % "continuous" funnel function
ref = Ref(t_con,t_change); % "continuous" reference 
y = z_con(:,1); % "continuous" output of the system
e = y - ref; % "continuous" error
u_ZoH = zeros(size(t_con,1),size(u_samp,2));
eta = z_con(:,2);

for k=1:size(t_con,1)
for l=1:floor(t_con(k)/tau)-1
    u_ZoH(k,:) = u_samp(l,:);
end
e1N(k) = norm(E(k,1));
e_norm(k) = norm(e(k,:));
end

figure;
plot(t_con,E(:,1),'b')
hold on
plot(t_con,E(:,2),'r')


%% Pure funnel control
  options = odeset(AbsTol=1e-6,RelTol=1e-6);
[t_FC,z_FC] = ode15s(@(t,x) funnel_control_ode(t,x,R1,R2,S,Gam,Q,P,a_psi,b_psi,c_psi,t_change),[0,tmax],X0,options);  % integrating the dynamics
y_FC = z_FC(:,1);
ref_FC = Ref(t_FC',t_change)';
e_FC = y_FC - ref_FC;
eFC_norm = zeros(size(t_FC));
for l = 1:size(t_FC,1)
    eFC_norm(l) = norm(e_FC(l,:));
end
%% Extracting the control u from the solution of the ode
[~,u_fc] = cellfun(@(t,x)  funnel_control_ode(t,x',R1,R2,S,Gam,Q,P,a_psi,b_psi,c_psi,t_change), num2cell(t_FC), num2cell(z_FC,2),'uni',0);
u_fc=u_fc';
u_FC = cell2mat(u_fc);

%% Vergleich der Schrittweiten und Controls
for l=1:size(t_FC,1)-1
    delta_tFC(l) = t_FC(l+1) - t_FC(l);
end

y_ZoH =y;
y_ZoH_m =y;
t_con_m = t_con;
ref_m = ref;
psi_m = psi;
u_ZoH_m =u_ZoH;
u_FC_m= u_FC;

if 1 == Scenario
    save('data_fc_zoh_scen1.mat', 'y_ZoH','t_con','ref','psi','u_ZoH','u_FC','t_FC','y_FC');
else
    save('data_fc_zoh_scen2.mat', 'y_ZoH_m','t_con_m', 'ref_m','psi_m','u_ZoH_m','u_FC_m','t_FC','y_FC');
end

figure('Name','Reference and funnel'); % Norm of the tracking error
plot(t_con,ref+psi,'k--')
hold on
plot(t_con,ref-psi,'k--')
plot(t_con,y,'r');
plot(t_FC,y_FC,'b')
legend('$\psi(t)$','$\|y(t) - y_{\rm ref}(t)\|$', ...
    '$\| y_{\rm FC}(t) - y_{\rm ref}(t) \|$',...
    'Interpreter','latex', 'FontSize',16)
box off

figure('Name','Controls'); % Controls
plot(t_con, u_ZoH,'r')
hold on
plot(t_FC,u_FC,'b')
%axis([0 tmax -15 15])
legend('$u(t)$',...
    '$ u_{\rm FC}(t)$',...
    'Interpreter','latex', 'FontSize',16)
box off

%% Functions

% ODE for sampled
function [dz,E] = sampled_ode(r,z,R1,R2,S,Gam,Q,P,u,a_psi,b_psi,c_psi,t_change)
dz = zeros(4,1);

y = z(1);
dy = z(2);
eta = z(3:4);

phi = Phi(r,a_psi,b_psi,c_psi);
ref = Ref(r,t_change);
dref = dRef(r,t_change);
e = y - ref;
de = dy - dref;
e1 = phi*e;
e2 = phi*de + Gain(norm(e1).^2) * e1;

E = [e1;e2];

% dynamics
dz(1) = z(2);
dz(2) = R1*y + R2*dy + S*eta + Gam .* u;
dz(3:4) = Q*eta + P*y;

disp(r)
end

% ODE for pure funnel control
function [dx,u_fc] = funnel_control_ode(t,x,R1,R2,S,Gam,Q,P,a_psi,b_psi,c_psi,t_change)
dx = zeros(4,1);

y = x(1);
dy = x(2);
eta = x(3:4);

phi = Phi(t,a_psi,b_psi,c_psi);
ref = Ref(t,t_change);
dref = dRef(t,t_change);
e = y - ref;
de = dy - dref;
e1 = phi*e;
e2 = phi*de + Gain(norm(e1).^2) * e1;

u_fc = - Gain(norm(e2).^2) * e2;

% dynamics
dx(1) = x(2);
dx(2) = R1*y + R2*dy + S*eta + Gam * u_fc;
dx(3:4) = Q*eta + P*y;


disp(t)
end


function r = Ref(t,t_change) % Reference
r = 0.4*sin(pi/2*t);
end

function dr = dRef(t,t_change)
dr = 0.4*pi/2*cos(pi/2*t);
end

function psi = Psi(t,a,b,c) % Funnel boundary
    psi = a*exp(-b*t) + c;
end

function phi = Phi(t,a,b,c) % Funnel function
phi = 1./Psi(t,a,b,c);
end

function dphi = dPhi(t,a,b,c)
dphi = a*b*exp(-b*t) .* Phi(t,a,b,c).^2 ;
end


function bet_f=Gain(e) % Bijection in funnel control
bet_f = 1/(1-e);
end

function dbet_f =dGain(s) % Bijection in funnel control
dbet_f = 1/(1-s)^2;
end

% Fehlervariablen ek
function fun = Ek(phi,e,e_pre)
fun = phi*e + Gain(norm(e_pre)^2) * e_pre;
end





