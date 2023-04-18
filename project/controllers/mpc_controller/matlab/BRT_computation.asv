function [safety_controller, worst_dist, data, tau, g, derivatives] = BRT_computation(params)
% unpack params
obsX1 = params.obsX1;
obsY1 = params.obsY1;
obsX2 = params.obsX2;
obsY2 = params.obsY2;
speed = params.speed;
wMax = params.wMax;
dMax = params.dMax;
width1 = params.obswidth1;
height1 = params.obsheight1;
width2 = params.obswidth2;
height2 = params.obsheight2;

%% TODO
% Define the grid for the computation: g
% g =...
grid_min = [-10; -10; -10]; % Lower corner of computation domain
grid_max = [10; 10; 10];    % Upper corner of computation domain
N = [75; 75; 75];         % Number of grid points per dimension
g = createGrid(grid_min, grid_max, N);


grid_min1 = [-1; -1]; % Lower corner of computation domain
grid_max1 = [1; 1];    % Upper corner of computation domain
N1 = [10; 10];         % Number of grid points per dimension
g1 = createGrid(grid_min1, grid_max1, N1);
%% TODO
% Define the failure set: data0
% data0 = ....


test1 = shapeRectangleByCorners(g,[obsX1; obsY1; -pi], [obsX1 + width1; obsY1 + height1; pi]);
test2 = shapeRectangleByCorners(g,[obsX2; obsY2; -pi], [obsX2 + width2; obsY2 + height2; pi]);

% test1 = shapeRectangleByCorners(g,[ obsX1, obsY1, -wMax], [obsY1 + height1, obsX1 + width1, wMax]);
% test2 = shapeRectangleByCorners(g,[obsY2,  obsX2, -wMax], [ obsY2 + height2, obsX2 + width2, wMax]);
test3 = shapeRectangleByCorners(g1,[0,0], [.05,.05]);
data0 = shapeUnion(test1,test2);

% time
t0 = 0;
tMax = 2;
dt = 0.05;
tau = t0:dt:tMax;

% control trying to min or max value function?
uMode = 'max';
dMode = 'min';

% Define dynamic system
dCar = DubinsCar([0, 0, 0], wMax, speed, dMax);

% Put grid and dynamic systems into schemeData
schemeData.grid = g;
schemeData.dynSys = dCar;
schemeData.accuracy = 'high'; %set accuracy
schemeData.uMode = uMode;
schemeData.dMode = dMode;
% HJIextraArgs.obstacles = obs;

%% Compute value function
% HJIextraArgs.visualize = true; %show plot
HJIextraArgs.visualize.valueSet = 1;
HJIextraArgs.visualize.initialValueSet = 1;
HJIextraArgs.visualize.figNum = 1; %set figure number
HJIextraArgs.visualize.deleteLastPlot = false; %delete previous plot as you update

% uncomment if you want to see a 2D slice
HJIextraArgs.visualize.plotData.plotDims = [1 1 0]; %plot x, y
HJIextraArgs.visualize.plotData.projpt = [0]; %project at theta = 0
HJIextraArgs.visualize.viewAngle = [0,90]; % view 2D

%[data, tau, extraOuts] = ...
% HJIPDE_solve(data0, tau, schemeData, minWith, extraArgs)
[data, tau2, ~] = ...
  HJIPDE_solve(data0, tau, schemeData, 'zero', HJIextraArgs);
derivatives = computeGradients(g, data(:,:,:,end));
safety_controller =  dCar.optCtrl([], [], derivatives, 'max');
worst_dist =  dCar.optDstb([], [], derivatives, 'min');
tau = tau2;

