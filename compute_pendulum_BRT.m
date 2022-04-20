clear all;

%% Grid
grid_min = [-pi, -8]; % Lower corner of computation domain8
grid_max = [pi, 8];    % Upper corner of computation domain
N = [101; 101;];         % Number of grid points per dimension
pdDims = 1;               % 3rd dimension is periodic
g = createGrid(grid_min, grid_max, N, pdDims);
% Use "g = createGrid(grid_min, grid_max, N);" if there are no periodic
% state space dimensions

%% target set
data0 = shapeUnion(shapeRectangleByCorners(g, [pi/4; -inf], [pi/2; inf]), ...
    shapeUnion(shapeRectangleByCorners(g, [-inf, 7], [inf, 8]), ...
                shapeRectangleByCorners(g, [-inf, -8], [inf, -7])));

%% time vector
t0 = 0;
tMax = 2.0;
dt = 0.1;
tau = t0:dt:tMax;

%% problem parameters
uMode = 'max';
% do dStep2 here

%% Pack problem parameters
params.u_min = -2;
params.u_max = 2;
params.l = 1.0;    % [m]        length of pendulum
params.m = 1.0;    % [kg]       mass of pendulum
params.g = 10; % [m/s^2]    acceleration of gravity
params.b = 0.08; % [s*Nm/rad] friction coefficient

obj = InvertedPendulum([0; 0], params);
% Put grid and dynamic systems into schemeData
schemeData.grid = g;
schemeData.dynSys = obj;
schemeData.accuracy = 'high'; %set accuracy
schemeData.uMode = uMode;

%% Compute value function
HJIextraArgs.visualize = true; %show plot
[data, tau2, ~] = ...
  HJIPDE_solve(data0, tau, schemeData, 'minVOverTime', HJIextraArgs);



dataTraj = flip(data,3);
save("pend_max_brt.mat", "g", "dataTraj", "tau2", "obj")

%%
dataEnd = data(:, :, end);
t0 = 0;
tMax = 0.15;
dt =0.05;
tau = t0:dt:tMax;
schemeData.grid = g;
schemeData.dynSys = obj;
schemeData.accuracy = 'high'; %set accuracy
schemeData.uMode = 'min';

%%
HJIextraArgs.visualize = true; %show plot
[data, tau2, ~] = HJIPDE_solve(dataEnd, tau, schemeData, 'minVOverTime', HJIextraArgs);

dataTraj = flip(data,3);
save("pend_brt_min_max.mat", "g", "dataTraj", "tau2", "obj")