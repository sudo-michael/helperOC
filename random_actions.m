clear all;
close all;
load("pend_brt_max_min.mat")

%set the initial state
xinit = [0; 1];

% time vector
t0 = 0;
tMax = 10.0;
dt = 0.05;
tau = t0:dt:tMax;

%check if this initial state is in the BRS/BRT
value = eval_u(g,dataTraj(:,:,1),xinit);

if value >= 0 %if initial state is in BRS/BRT
    % find optimal trajectory
    obj.x = xinit; %set initial state of the dubins car
    
    TrajextraArgs.uMode = 'max'; %set if control wants to min or max
    TrajextraArgs.visualize = false; %show plot
    TrajextraArgs.fig_num = 1; %figure number
    %we want to see the first two dimensions (x and y)
    TrajextraArgs.projDim = [1 1];
    
    [traj, traj_tau, u_hist, v_hist] = randomTraj(g, dataTraj, tau, obj, TrajextraArgs);
    %     u = computeOptTraj_u(g, dataTraj, tau2, obj)
else
    error(['Initial state is not in the BRS/BRT! It have a value of ' num2str(value,2)])
end
traj
figure,
plot(traj_tau, traj(1, :), 'r'); % theta
title('theta')
figure,
plot(traj_tau, traj(2, :), 'b'); % theta dot
title('theta dot')

figure,
plot(traj_tau, u_hist(1, :), 'b'); 
title('u')


figure,
plot(traj_tau, v_hist(1, :), 'b'); 
title('v')
