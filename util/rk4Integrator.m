%% Runge-Kutte 4 Integrator
function X = rk4Integrator(ode,t0,tf,M,x, cb_u)
    h=(tf-t0)/M;
    xnext=x;
    if exist('cb_u','var')
        for i=1:M
            xnext=rk4_with_cb(ode,h,t0+i*h,xnext,cb_u,i);
            X(:,i)=xnext;
        end
    else
        for i=1:M
            xnext=rk4(ode,h,t0+i*h,xnext);
            X(:,i)=xnext;
        end
    end
end

function xf = rk4_with_cb(ode,h,t,x, u_callback, i)
    k1 = ode(t,x,u_callback(t,x,i));
    k2 = ode(t+h/2,x+h/2*k1,u_callback(t+h/2,x+h/2*k1,i));
    k3 = ode(t+h/2,x+h/2*k2,u_callback(t+h/2,x+h/2*k2,i));
    k4 = ode(t+h,x+h*k3,u_callback(t+h,x+h*k3,i));
    xf = x + h/6 * (k1 + 2*k2 + 2*k3 + k4);
end