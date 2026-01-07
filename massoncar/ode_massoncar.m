%State [y, yd, eta1, eta2]
function dz = ode_massoncar(Time, State, ControlValue)
    y = State(1);
    dy = State(2);
    eta = State(3:4);
    
    R1 =0;
    R2 = 8/9;
    S = -4*sqrt(2)/9* [2 1];
    Gam =1/9;
    Q =[0 1;
        -4 -2];
    P = 2*sqrt(2)*[1;0];
    dz = [dy;
          R1*y + R2*dy + S*eta + Gam .* ControlValue;
          Q*eta + P*y];
end

