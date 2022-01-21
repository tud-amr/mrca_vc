function xnext = single_integrator_discrete_dynamics(z, p)

    global index
    
    % state, control and dt
    x = z(index.z.pos);
    u = z(index.z.inputs);
    dt = p(index.p.dt);
    
    % propogation
    xnext = x + dt*u;

end
