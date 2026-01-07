function y = ode_massoncar_outputdot_orig(State)
    m_1     = 4;
    m_2     = 1;
    k       = 2;
    d       = 1;
    teta    = pi/4;
    
    mu      = m_2*(m_1+m_2*(sin(teta)^2));
    mu_1    = m_1/mu;
    mu_2    = m_2/mu;

   A = [0 1 0 0;...
     0 0 mu_2*k*cos(teta) mu_2*d*cos(teta);...
     0 0 0 1;...
     0 0 -(mu_1 + mu_2)*k  -(mu_1 + mu_2)*d];
    
    C = [1 0 cos(teta) 0];
    y = C*A*State;
end

