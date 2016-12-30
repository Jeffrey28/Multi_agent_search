% set up for the main program

%% %%%%%%% General Setup %%%%%%%%%%
set(0,'DefaultFigureWindowStyle','docked')
set(0,'defaultAxesFontName', 'Times New Roman')
set(0,'defaultTextFontName', 'Times New Roman')

save_data = true; % save all sim data
show_plot = true; % draw plots
save_plot = true; % save plots and corresponding .mat file 
sim_mode = true;

sim_len = 30;%50; % max step
% rounds of consensus at each time step
cons_step = 10;
cons_fig = false; % whether to show intermediate step of consensus

% Setup for multiple trials
trial_num = 10; % 10 % number of trials to run

% select the motion of agents and target
selection = 4;
switch selection
    case 1,  r_move= 0; tar_move=0;
    case 2,  r_move= 0; tar_move=1;
    case 3,  r_move= 1; tar_move=0;
    case 4,  r_move= 1; tar_move=1;
    otherwise, error('No selection.');
end

% the set of robots whose pdf will be drawn
if r_move == 0
    sim_r_idx = [1,3,5];
else
%     sim_r_idx = [1,3,5];
    sim_r_idx = 4;
end

% the sensor type of each robot
sensor_set_type = 'htr'; %'brg':bearing ,'ran':range, 'rb':range-bearing, 'htr':heterogeneous
switch sensor_set_type
    case 'brg'
        sensor_set = {'brg','brg','brg','brg','brg','brg'};
    case 'ran'
        sensor_set = {'ran','ran','ran','ran','ran','ran'};
    case 'rb'
        sensor_set = {'rb','rb','rb','rb','rb','rb'};
    case 'htr'
%         sensor_set = {'brg','ran','rb','brg','ran','rb'};
        sensor_set = {'brg','brg','brg','ran','ran','ran'};
end

num_robot = 6;

dt = 1; % discretization time interval

fld_size = [100;100];  % Field size

filter_type = [1 0 0]; % 1: turn on the corresponding filters. [dbf,cons,cent]

%% define target and robot positions
% used for initialization, don't need to call again unless necessary
%{
% robot initialization
rbt(NumOfRobot) = struct;
% these are randomly generated initial positions of robots
% x_set = [20,40,60,80,60,40];
% y_set = [50,20,20,50,80,80];

% generate random initial positions of robots
for ii = 1:NumOfRobot
    rnd_rbt_pos(1,:) = randi([5,fld.x-5],1,trial_num);
    rnd_rbt_pos(2,:) = randi([5,fld.y-5],1,trial_num);
    rbt(ii).init_pos = rnd_rbt_pos;
end

% generate the covariance matrices and offset of sensors
for ii = 1:NumOfRobot
    rbt(ii).sen_cov = randCov(2,fld.x,fld.y); % covariance of Gaussian distribution
    rbt(ii).inv_sen_cov = eye(2)/rbt(ii).sen_cov; % inverse of covariance
    rbt(ii).sen_offset = randn(2,1).*[5;5]; % offset of the center of Gaussian distribution
end

save('sensor_spec.mat','rbt');


% target initialization

% generate random target position (just generate once and use them afterwards)
% rnd_tar_pos(1,:) = randi([5,fld.x-5],1,trial_num);
% rnd_tar_pos(2,:) = randi([5,fld.y-5],1,trial_num);
%}

% these are randomly generated sensor specifications
% load the variable using the name that I want
rbt_spec = load('rbt_spec.mat');
rbt_spec = rbt_spec.rbt;

% these are randomly generated target positions from [20,80] at each
% direction
tx_set = [68 36 24 80 48 29 42 44 54 68];
ty_set = [33 42 43 64 69 65 32 23 79 20];

% tx_set = [68, 55, 41, 10, 75, 35, 60, 72, 14, 16];
% ty_set = [55, 49, 86, 77, 71, 9, 11, 13, 77, 90];

% radius, period and moving direction (clockwise or counterclockwise) of sensor circular trajectory
r_set = [5 10 7 10 15 7];
T_set = [20,15,30,25,15,20];
dir_set = [1 -1 1 -1 -1 -1];

% communication neighbor
rbt_nbhd = {[2,6],[1,3],[2,4],[3,5],[4,6],[1,5]}; %{[],[],[],[],[],[]};
            
mode_num = 4;

u_set = [[1;1],[-1;-1],[1;-1],[-1;1]]; %inPara.u_set; 
V_set = 0.01*eye(2);%

% load update matrices
if exist('upd_matrix','var') == 0
    load('upd_matrix_v001.mat','upd_matrix');
end

%% Compute the sensor probility matrix
% just a placeholder now, will see how fast the program will run without
% this precomputation