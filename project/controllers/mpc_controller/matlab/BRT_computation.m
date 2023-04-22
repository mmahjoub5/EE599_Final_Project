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

%add 4th dimension for these %
grid_min = [-110; -110; - 2 * pi; -50]; % Lower corner of computation domain
grid_max = [110; 110; 2 * pi; 50];    % Upper corner of computation domain
N = [50; 50; 50; 50];         % Number of grid points per dimension
g = createGrid(grid_min, grid_max, N, 3);

%% TODO
% Define the failure set: data0
% data0 = ....

wMax = pi;
rect1 = shapeRectangleByCorners(g, [-105; -105; -pi; -40], [45; 45; pi; 40]);%add velcoity dimension - neg max speed - + max speed
rect2 = shapeRectangleByCorners(g, [-80; -80; -pi; -40], [20; 20; pi; 40]);
rect3 = shapeDifference(rect1, rect2);


rect4 = shapeRectangleByCorners(g, [-50; -50; -pi;-40], [105; 105; pi; 40]);
rect5 = shapeRectangleByCorners(g, [-22; -22; -pi; -40], [80; 80; pi; 40]);
rect6 = shapeDifference(rect4, rect5);


data0 = shapeUnion(rect3, rect6);
data0 = shapeComplement(data0);



% time
t0 = 0;
tMax = 2.0;
dt = 0.05;
tau = t0:dt:tMax;

disp('choose max or min')
% control trying to min or max value function?
uMode = 'max';
dMode = 'min';

% Define dynamic system
%dCar = DubinsCar([0, 0, 0], wMax, speed, dMax);
% create plne 4d 
dCar = Plane4D([0, 0, 0, 0], wMax, [-speed, speed], [-dMax, dMax]);

% Put grid and dynamic systems into schemeData
schemeData.grid = g;
schemeData.dynSys = dCar;
schemeData.accuracy = 'high'; %set accuracy
schemeData.uMode = uMode;
schemeData.dMode = dMode;
% HJIextraArgs.obstacles = obs;

%% Compute value function
disp('compute value function')
% HJIextraArgs.visualize = true; %show plot
HJIextraArgs.visualize.valueSet = 1;
HJIextraArgs.visualize.initialValueSet = 1;
HJIextraArgs.visualize.figNum = 1; %set figure number
HJIextraArgs.visualize.deleteLastPlot = false; %delete previous plot as you update

% uncomment if you want to see a 2D slice
HJIextraArgs.visualize.plotData.plotDims = [1 1 0 0]; %plot x, y
HJIextraArgs.visualize.plotData.projpt = [0 0]; %project at theta = 0
HJIextraArgs.visualize.viewAngle = [0,90]; % view 2D

%[data, tau, extraOuts] = ...
% HJIPDE_solve(data0, tau, schemeData, minWith, extraArgs)
[data, tau2, ~] = ...
    HJIPDE_solve(data0, tau, schemeData, 'zero', HJIextraArgs);
%%
derivatives = computeGradients(g, data(:,:,:,:,end));
safety_controller =  dCar.optCtrl([], [], derivatives, 'max');
worst_dist =  dCar.optDstb([], [], derivatives, 'min'); % save this as well and then load it again 
tau = tau2;

