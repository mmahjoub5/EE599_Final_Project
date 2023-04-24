function y = fourDim_dynamics(x,y,theta, v, w, a, dt, dist)
    x_t = x + dt*v*cos(theta) + dist(1);
    y_t = y + dt * v * sin(theta) + dist(2);
    theta_t = theta + dt * w;
    v_t = v + dt * a;
    y =[x_t, y_t, theta_t, v_t];
end
