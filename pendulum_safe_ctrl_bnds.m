function [low, high] = pendulum_safe_ctrl_bnds(xinit, vars)
%     vars = load("/local-scratch/localhome/mla233/Downloads/Pendulum/Pendulum/pend_brt.mat");
%     xinit
    
    l = vars.obj.l;    % [m]        length of pendulum
    m = vars.obj.m;    % [kg]       mass of pendulum
    g = vars.obj.g; % [m/s^2]    acceleration of gravity
    b = vars.obj.b; % [s*Nm/rad] friction coefficient
    brt = vars.dataTraj(:, :, 1);
    value = eval_u(vars.g, brt, vars.obj.x);
    Deriv = computeGradients(vars.g, brt);
    deriv = eval_u(vars.g, Deriv, vars.obj.x);
    % keep if we can't find range of contorl
    u = vars.obj.optCtrl(1, vars.obj.x, deriv, 'max');

    pVptheta = deriv(1);
    pVptheta_dot = deriv(2);

    theta = xinit(1);
    theta_dot = xinit(2);

    f1 = theta_dot;
    f2 = (-b * theta_dot + m * g * l * sin(theta)/2 ) / (m * l^ 2/3);
    g1 = 0;
    g2 = -1 / (m * l ^ 2/3);

    gradVTf = @(u_input) pVptheta * f1 + pVptheta_dot * f2 +  pVptheta_dot * g2 * u_input;

    % u_range = -2:0.1:2
    % out = gradVTf(u_range)
    % plot(u_range, out)

    % check if slope is positive or negative
    multiplier = deriv(1).*g1 + deriv(2).*g2;

    % disp(multiplier)
    x_intercept = -(pVptheta * f1 + pVptheta_dot * f2) / (pVptheta_dot * g2);
    % TODO: get from vars
    U_MIN = -2;
    U_MAX = 2;
    x_intercept = min(max(x_intercept, U_MIN), U_MAX);

    if sign(multiplier) < 0
        safe_ctrl_bnd = [-2, x_intercept];
    elseif sign(multiplier) > 0
        safe_ctrl_bnd = [x_intercept, 2];
    else
        safe_ctrl_bnd = [u, u];
    end

    % safe_ctrl_bnd
    % gradVTf(x_intercept)
    low = safe_ctrl_bnd(1);
    high = safe_ctrl_bnd(2);
end