function F = ball_ctrl(in,P)
    z_r         = in(1);
    z           = in(2);
    theta       = in(3);
    zdot        = in(4);
    thetadot    = in(5);
    t           = in(6);
    
    gamma1 = 30;
    gamma2 = -2;
    gamma3 = -10;
    Gamma3 = 1;
    s = thetadot + gamma1*theta + gamma2*zdot + gamma3*(z-z_r);

    u = (2*P.m*z*zdot + P.k2)*thetadot...
        + (P.m*P.g*z + (P.L/2)*P.M*P.g)*cos(theta)...
        - (gamma1*thetadot...
            + gamma3*zdot...
            + gamma2*(z*thetadot^2 - P.g*sin(theta))/P.k4)*(P.m*z^2 + P.k1)...
        - (P.m*z^2 + P.k1)*Gamma3 * sign(s);
    F = u;
end