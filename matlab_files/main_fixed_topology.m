%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Multi Sensor-based Distributed Bayesian Estimator
% This code is for distributed bayesian estimator for target positioning and tracking
% (1) Target: Static target with unknown position
% (2) Sensors: Binary sensor with only 0 or 1 measurement
% (3) Strategy: (3.1) Observation Exchange strategy (Neighbourhood or Global-Exchange)
%               (3.2) Probability Map Consensus strategy (single step or multi-step)
%% 2015 June; All Copyright Reserved
% Fixed topology case
% 3/14/2016
% original paper got rejected from ICRA 16 and T-ASE. program is modified
% for CDC 16
% compare the LIFO-DBF with consensus method and centralized method
% 5/4/16
% re-write the fixed-topology case in OOP. Hopefully this will make my
% coding cleaner, more readable and easier to adapt

% possible bug for this program:
% 1. in moving target case, maybe we should first let target move, then observe, then update pdf.
% 2. try to change the update step, current method takes huge memory and
% may have bugs.

% progress on 5/17/16
% debugging the code for static robot static target case. Needs to continue
% debugging and then debug for moving robot or target cases.

% main function for running the simulation
clear; clc; close all

%% %%%%%%%%%%%%%%%%%%%%%% General Setup %%%%%%%%%%%%%%%%%%%%%%
save_data = false; % save data or not
show_plot = false; % draw plots or not

max_step = 50; % max step
% rounds of consensus at each time step
cons_step=10;

% select the motion of agents and target
selection = 1;
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
    sim_r_idx = [2,4,6];
end

num_robot = 6;

dt = 1; % discretization time interval

%% %%%%%%%%%%%%%%%%%%%%%% Simulation %%%%%%%%%%%%%%%%%%%%%%
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

% these are randomly generated target positions
tx_set = [68, 55, 41, 10, 75, 35, 60, 72, 14, 16];
ty_set = [55, 49, 86, 77, 71, 9, 11, 13, 77, 90];

% communication neighbor
rbt_nbhd = {[2,6],[1,3],[2,4],[3,5],[4,6],[1,5]};

% Setup for multiple trials
trial_num = 1; % 10 % number of trials to run

for trial_cnt = 1:trial_num
    % initialize field class
    fld_size = [100;100];  % Field size
    target.pos = [tx_set(trial_cnt);ty_set(trial_cnt)];
    target.u_set = {ones(2,1),-ones(2,1)};
    target.V_set = {0.01*eye(2),0.01*eye(2)}; % Covariance of process noise model for the target
    target.model_idx = 1;
    target.traj = [];
    
    inPara_fld = struct('fld_size',fld_size,'target',target,'tar_move',tar_move,'dt',dt);
    fld = Field(inPara_fld);    
%     fld = fld.setMotionModel();
            
    % initialize robot class
    inPara_rbt = struct('num_robot',num_robot,'dt',dt);
    rbt = cell(num_robot,1);
    if r_move == 0
        for rbt_cnt = 1:num_robot
            inPara_rbt = struct;
            inPara_rbt.pos = rbt_spec(rbt_cnt).init_pos(:,trial_cnt);
            inPara_rbt.sen_cov = rbt_spec(rbt_cnt).sen_cov;
            inPara_rbt.inv_sen_cov = rbt_spec(rbt_cnt).inv_sen_cov;
            inPara_rbt.sen_offset = 0; %rbt_spec(rbt_cnt).sen_offset;
            inPara_rbt.fld_size = fld_size;
            inPara_rbt.max_step = max_step;
            inPara_rbt.nbhd_idx = rbt_nbhd{rbt_cnt};
            inPara_rbt.r_move = r_move;
            inPara_rbt.num_robot = num_robot;
            inPara_rbt.idx = rbt_cnt;
            rbt{rbt_cnt} = Robot(inPara_rbt);
        end
    elseif r_move == 1
        for rbt_cnt = 1:num_robot
            inPara_rbt = struct;
            inPara_rbt.center = rbt_spec.init_pos(:,rbt_cnt);
            inPara.r = 15;
            inPara_rbt.fld_size = fld_size;
            inPara_rbt.max_step = max_step;
            inPara_rbt.nbhd_idx = rbt_nbhd{rbt_cnt};
            rbt(rbt_cnt) = Robot(inPara_rbt);
            inPara_pred.u_set = target.u_set;
            inPara_pred.V_set = target.V_set;
            rbt = rbt.predStep(inPara_pred);
        end
    end
    %% %%%%%%%%%%%%%% main code of simulation %%%%%%%%%%%%%%%%%%
    %% target moves
    fld.targetMove;
    
    %% filtering
    %% LIFO-DBF
    % Bayesian Updating steps:
    % (1) observe and update the stored own observations at time k
    % (2) exchange and update stored observations
    % (3) update probability map
    % (4) repeat step (1)
    for ii = 1:num_robot
        % step 1
        % observe
%         rbt(ii) = rbt(ii).sensorGen(fld); % simulate the sensor measurement  
        rbt{ii} = rbt{ii}.sensorGen(fld);
        
        % update own observation
        inPara1 = struct('selection',selection);
        rbt{ii} = rbt{ii}.updOwnMsmt(inPara1);
        
        % step 2
        inPara2 = struct;
        inPara2.selection = selection;        
        inPara2.rbt_nbhd_set = rbt(rbt{ii}.nbhd_idx);
        rbt{ii} = rbt{ii}.dataExch(inPara2);
        
        % step 3
        inPara3 = struct('selection',selection);
        rbt{ii} = rbt{ii}.DBF(inPara3);
    end
            
    %% Concensus
    
    %% Centralized 
    
    %% go to next iteration
    for ii = 1:num_robot
       rbt{ii} = rbt{ii}.stepUpdate; 
    end
end

%% %%%%%%%%%%%%%%%%%%%%%% Simulation Results %%%%%%%%%%%%%%%%%%%%%%
% initialize sim class 
inPara = struct;
inPara_sim.r_move = r_move;
inPara_sim.tar_move = tar_move;
inPara_sim.max_step = max_step;
inPara_sim.cons_step = cons_step;
inPara_sim.num_robot = num_robot;
inPara_sim.r_init_pos_set = extractfield(rbt_spec,'init_pos');
inPara_sim.sim_r_idx = sim_r_idx;
inPara_sim.trial_num = trial_num;
inPara_sim.dt = dt;

sim = Sim(inPara_sim);

% draw plot
if show_plot
    sim.plotStim();
end

% save data
if save_data 
    sim.saveSimData(selection);
end
