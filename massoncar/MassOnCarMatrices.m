%System parameter for y'' = R1*y + R2 y' + S*eta + Gam*u, eta' = Q*eta + P y
function [R1_double,R2_double,S_double,Gam_double,Q_double,P_double] = MassOnCarMatrices(m1_val, m2_val, k_val, d_val)
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

VAL = [m1_val m2_val k_val d_val]; % m1 m2 k d

Gam_num = subs(Gam,[m1 m2 k d],VAL) ;
R1_num = subs(R1,[m1 m2 k d],VAL)  ;
R2_num = subs(R2,[m1 m2 k d],VAL)  ;
S_num = subs(S,[m1 m2 k d],VAL)  ;

Q_num = subs(Q,[m1 m2 k d],VAL) ;
P_num = subs(P,[m1 m2 k d],VAL) ;


R1_double = double(R1_num);
R2_double= double(R2_num);
S_double= double(S_num);
Gam_double= double(Gam_num);
Q_double= double(Q_num);
P_double= double(P_num);

%U_num = subs(U,[m1 m2 k d],VAL) ;
end