% clear all;
% close all;
% clc;
% 
%
% I_1     = 0.136;
% I_2     = 0.12;
% k       = 10;
% d       = 16;
% 
% M = [1 0 0 
%     0 I_1 0
%     0 0 I_2];

function [tau,u_max] = estimate_tau(I_1,I_2,k,d)

M = [1 0 0 
     0 I_1 0
     0 0 I_2];

M_inv = M\eye(size(M));

A_bar = [0 1 -1
         -k -d d
          k d -d];

B_bar = [0;1;0];

A = M_inv * A_bar ;
B = M_inv * B_bar;

C = [0 1 0];

Q = 1/I_2*[0, I_2; -k, -d] ;

K = lyap(Q',eye(2)) ;
K_inv = K\eye(2);

M_K = sqrt(norm(K,2)*norm(K_inv,2)) ;
mu_K = 1/(2*norm(K,2)) ;

P = I_2*[-I_2;d];

ref = 250;
dref = 250/(sqrt(2*pi));
psi = 25;

eta = M_K/mu_K*norm(P,2)*(ref + psi) ;

R = -d/I_1;
S = 1/I_1*[k,d];

fmax = (norm(R,2) + norm(S,2)*norm(P,2)*M_K/mu_K)*(ref + psi) ;

kappa_0 = (fmax + dref)/psi ;

gam = C*B;

bet = 2*kappa_0*psi/gam ;

%lambda=0.5;
%bet = round(bet,1)
%bet = 15;


kappa_1 = kappa_0 + 2*gam*bet/psi;

tau = min(kappa_0/kappa_1^2,1/(2*kappa_0));

u_max = 2*bet;

end
