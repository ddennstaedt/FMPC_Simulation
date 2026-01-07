function y = ode_massoncar_output_alt(State)

    teta    = pi/4;


    C = [1 0 cos(teta) 0];
    y = C*State;
end

