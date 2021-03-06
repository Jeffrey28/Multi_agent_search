% set up for the main program for using experiment data

%% %%%%%%% General Setup %%%%%%%%%%
set(0,'DefaultFigureWindowStyle','docked')
set(0,'defaultAxesFontName', 'Times New Roman')
set(0,'defaultTextFontName', 'Times New Roman')


sim_mode = false; % used in main_tv_topo.m, use simulated data instead of experiment data (no experiment in tv_topo though)
exp_mode = ~sim_mode;
save_data_exp = false; % save all sim data
show_plot = true; % draw plots
save_plot = true; % save plots and corresponding .mat file 
DBF_only = true; % only run DBF (true) or run DBF, CF and ConF (false)
DBF_type = 'hist'; % choose the implementation of dbf: particle filter('pf') or histogram('hist')
save_video = true; % save the progress figures to a video. When using this, don't use dual monitors, which can cause problem in the captured video region
save_data = false; % save all sim data. used in main_tv_topo.m and Sim.m and for drawing metrics plot
comp_metric = false; % decide if needs to compute metrics and compare them. used in main_tv_topo.m
comp_metric_pf = false; % decide if needs to compute metrics and compare them for pf implementation. used in main_tv_topo.m


sim_len = 40;%50; % max step
% rounds of consensus at each time step
cons_step = 10;
cons_fig = false; % whether to show intermediate step of consensus

% Setup for multiple trials
trial_num = 1; % 10 % number of trials to run

% select the motion of agents and target
selection = 3;
r_move = 2; % use experiment positions
tar_move = 0;

sim_r_idx = 1:3;

% the sensor type of each robot
sensor_set_type = 'sonar'; %'brg':bearing ,'ran':range, 'rb':range-bearing, 'htr':heterogeneous, 'sonar': sonar on p3dx
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
    case 'sonar'
        sensor_set = {'snr','snr','snr'};
end

num_robot = 3;

% all length unit in the code is meter. scale is to change the resolution 
% of the unit. e.g. scale=10 means to use 0.1 meter as the length unit
scale = 10; 

dt = 1; % discretization time interval

fld_size = floor([250.7;358]*2.54/100*scale);  % Field size

filter_type = [1 1 1]; % 1: turn on the corresponding filters. [dbf,cons,cent]

tx_set = [123.5]*2.54/100*scale;
ty_set = [122]*2.54/100*scale;

%% %%% load measurement data

%% define robot
% these data is not used for experiment analysis. include it just to make the
% main_fixed_topology work
rbt_spec = load('rbt_spec.mat');
rbt_spec = rbt_spec.rbt;

% communication neighbor
topo_select = 8; % topo for experiment data
switch topo_select
    case 8
        rbt_nbhd = {{[],[],[2]};
            {[],[1],[3]};
            {[],[1],[]}};            
        top_idx_set = [1 1 1 1 1 2 2 2 2 2 3 3 3 3 3];
end

% rbt_nbhd = {[2],[1,3],[2]}; %{[],[],[],[],[],[]};

particles = []; % this is in consistent with the code of main_tv_topo. Actually PF is not used here.

%% define target
% load update matrices
if exist('upd_matrix','var') == 0
    load('upd_matrix_v001.mat','upd_matrix');
end

target_mode = 'exp';
mode_num = 4;

u_set = [[1;1],[-1;-1],[1;-1],[-1;1]]; %inPara.u_set; 
V = 0.01*eye(2);%
            
% the data in meas_data is already in the unit of meter
if exist('meas_data','var') == 0
    load('exp_meas_data.mat','meas_data');
end

% sonar modeling
% y = B'*[x;1]
B = [0.9945;-0.1652];

% rescale data in meas_data
% meas_data: [x,y,theta,distance]
for ii = 1:num_robot
    meas_data{ii}(:,[1,2]) = meas_data{ii}(:,[1,2])*scale;
    switch ii
        case 1
            offset = 0*2.5/100;
        case 2
            offset = 4*2.5/100;
        case 3
            offset = 3.7*2.5/100;
    end
    meas_data{ii}(:,4) = ((B'*[meas_data{ii}(:,4),ones(size(meas_data{ii},1),1)]'+offset)*scale)';
end

meas_data{1} = meas_data{1}(end:-1:1,:);

samp_num = sim_len; % in each robot, use this number of measurements
samp_meas_data = cell(3,1);
for ii = 1:3
    samp_idx = floor(linspace(1,size(meas_data{ii},1),samp_num));
    samp_meas_data{ii} = meas_data{ii}(samp_idx,:);
end

rbt_init_state = zeros(3,num_robot);
for ii = 1:num_robot
    rbt_init_state(1:2,ii) = (samp_meas_data{ii}(1,1:2))';
    rbt_init_state(3,ii) = samp_meas_data{ii}(1,3);
end

