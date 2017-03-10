% set up for the main program
% manual:
% after running each different target model (e.g., line, sinusoidal,
% circle), delete upd_matrix in the workspace.

%% %%%%%%% General Setup %%%%%%%%%%
set(0,'DefaultFigureWindowStyle','docked')
set(0,'defaultAxesFontName', 'Times New Roman')
set(0,'defaultTextFontName', 'Times New Roman')

sim_mode = true; % used in main_tv_topo.m, use simulated data instead of experiment data (no experiment in tv_topo though)
show_plot = true; % draw pdf at each step. used in main_tv_topo.m
save_plot = false; % save pdf of selected steps and corresponding .mat file. used in main_tv_topo.m and Sim.m (plotSim)
DBF_only = true; % only run DBF (true) or run DBF, CF and ConF (false)
save_video = true; % save the progress figures to a video. When using this, don't use dual monitors, which can cause problem in the captured video region
save_data = false; % save all sim data. used in main_tv_topo.m and Sim.m and for drawing metrics plot
comp_metric = false; % decide if needs to compute metrics and compare them. used in main_tv_topo.m

% decide which plot I want to draw. This is a shortcut to set up the
% parameters. If I choose both process_plot and metrics_plot to be false,
% then the parameter is manually chosen.
process_plot = false;
metrics_plot = false;

if process_plot
    show_plot = true;
    save_plot = true;
    DBF_only = false;
    save_data = false;
    comp_metric = false;
%     r_init_pos = [[35;73],[50;77],[21;26],[15;15],[46;17],[77;58]];
end

if metrics_plot
    show_plot = false;
    save_plot = false;
    DBF_only = false;
    save_data = true;
    comp_metric = true;
end

sim_len = 50; % max step
% rounds of consensus at each time step
cons_step = 10;
cons_fig = false; % whether to show intermediate step of consensus

% Setup for multiple trials
trial_num = 10; % 10 % number of trials to run % note: when trial_num =1, error occurs when computing the performance metrics

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
    sim_r_idx = 3;
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

filter_type = [1 1 1]; % 1: turn on the corresponding filters. [dbf,cons,cent]

%% define robot
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
rbt_spec = rbt_spec.rbt; % gives the center of the circular motion

% tx_set = [68, 55, 41, 10, 75, 35, 60, 72, 14, 16];
% ty_set = [55, 49, 86, 77, 71, 9, 11, 13, 77, 90];

% radius, period and moving direction (clockwise or counterclockwise) of sensor circular trajectory
r_set = [20 10 15 10 15 10]; %[5 10 7 10 15 7];
T_set = [20,15,30,25,15,20];
dir_set = [1 -1 1 -1 -1 -1];

% communication neighbor. note that the neighbors here are inbound
% neighbors, i.e., the ones a UGV can receive info from
topo_select = 6;

switch topo_select
    case 1
        rbt_nbhd = {{[2,6],[]};
            {1,3};
            {4,2};
            {[3,5],[]};
            {4,6};
            {1,5}};
    case 2
        rbt(1).top(1).neighbour=6;
        rbt(2).top(1).neighbour=0;
        rbt(3).top(1).neighbour=5;
        rbt(4).top(1).neighbour=0;
        rbt(5).top(1).neighbour=3;
        rbt(6).top(1).neighbour=1;
        
        rbt(1).top(2).neighbour=2;
        rbt(2).top(2).neighbour=1;
        rbt(3).top(2).neighbour=0;
        rbt(4).top(2).neighbour=5;
        rbt(5).top(2).neighbour=4;
        rbt(6).top(2).neighbour=0;
        
        rbt(1).top(3).neighbour=0;
        rbt(2).top(3).neighbour=3;
        rbt(3).top(3).neighbour=2;
        rbt(4).top(3).neighbour=0;
        rbt(5).top(3).neighbour=6;
        rbt(6).top(3).neighbour=5;
    case 3 % fixed topology
        rbt_nbhd = {{[2,6]};
            {[1,3]};
            {[4,2]};
            {[3,5]};
            {[4,6]};
            {[1,5]}};
    case 4 % directed graph
        rbt_nbhd = {{2,[]};
            {3,[]};
            {4,[]};
            {[],5};
            {[],6};
            {[],1}};
    case 5 
        rbt_nbhd = {{6};
            {1};
            {2};
            {3};
            {4};
            {5}};
    case 6
        % topology that contains both unidirectional and bidirecational
        % communication, used for simulation in the paper
        rbt_nbhd = {{[2],[],[],[]};
            {[1,3],[],[],[]};
            {[2],[],[],[5,6]};
            {[],[3],[],[]};
            {[],[],[4],[]};
            {[],[1],[],[]}};
        top_idx_set = [1 2 2 2 4 1 3 1 3 4];    
end

%% define target 
target_mode = 'sin'; %%!!! warning: whenever change the target_mode, remember to delete the previous upd_matrix first

if strcmp(target_mode, 'linear')    
    % linear target model
    mode_num = 4;
    u_set = [[1;1],[-1;-1],[1;-1],[-1;1]]; %inPara.u_set;
    V = 0.1*eye(2);%
    % load update matrices
    if exist('upd_matrix','var') == 0
        load('upd_matrix_v01.mat','upd_matrix');
    end
    % these are randomly generated target positions from [20,80] at each
    % direction
    tx_set = [68 36 24 80 48 29 44 42 54 68 24];
    ty_set = [33 42 43 64 69 65 23 32 79 20 43];
    
elseif strcmp(target_mode, 'sin')
    % sinusoidal target model
    mode_num = 4;
    u_set = [[-1;1],[1;1],[-1;-1],[1;-1]];
    V = 0.1*eye(2);
    % load update matrices
    if exist('upd_matrix','var') == 0
        load('upd_matrix_sin_v01.mat','upd_matrix');
    end    
    % these are randomly generated target positions from [20,80] at each
    % direction
    tx_set = [69,27,58,36,78,29,78,68,45,68,58];
    ty_set = [75,75,25,53,78,79,49,28,75,78,25];
    
elseif strcmp(target_mode, 'circle') 
    mode_num = 5;
    center_set = [[50.5;50.5],[50.5;150.5],[50.5;-150.5],[-50.5;50.5],[150.5;50.5]]; %swtich the center of the circle
    V = 0.1*eye(2);%
    if exist('upd_matrix','var') == 0
        load('upd_matrix_cir_v01.mat','upd_matrix');
    end
    % these are randomly generated target positions from [20,80] at each
    % direction
    tx_set = [60,71,61,65,59,63,36,25,62,77,61];
    ty_set = [22,76,66,43,30,21,22,70,39,22,66];
end