function [u, value] = pendulum_opt_ctrl(xinit, vars)
%     vars = load("/local-scratch/localhome/mla233/Downloads/Pendulum/Pendulum/pend_brt.mat");
%     xinit
    vars.obj.x = xinit;
    brt = vars.dataTraj(:, :, 1);
    value = eval_u(vars.g, brt, vars.obj.x);
    Deriv = computeGradients(vars.g, brt);
    deriv = eval_u(vars.g, Deriv, vars.obj.x);
%     sprintf("deriv1: %f", deriv(1))
    u = vars.obj.optCtrl(1, vars.obj.x, deriv, 'max');
end

