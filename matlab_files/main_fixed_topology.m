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

% progress on 7/14/16
% has debugged the code for static robot static target case. Needs to write up the Sim class 
% and then debug for moving robot or target cases. Still writing the Sim
% class compareMetrics method

% main function for running the simulation
clear; clc; close all

%% %%%%%%% Simulation %%%%%%%%%%

% define parameters, precompute certain quantities
simSetup();

% initialize sim class
inPara = struct;
inPara_sim.r_move = r_move;
inPara_sim.tar_move = tar_move;
inPara_sim.sim_len = sim_len;
inPara_sim.cons_step = cons_step;
inPara_sim.num_robot = num_robot;
inPara_sim.r_init_pos_set = extractfield(rbt_spec,'init_pos');
inPara_sim.sim_r_idx = sim_r_idx;
inPara_sim.trial_num = trial_num;
inPara_sim.dt = dt;
inPara_sim.selection = selection;
sim = Sim(inPara_sim);

for trial_cnt = 1:trial_num
    % initialize field class    
    target.pos = [tx_set(trial_cnt);ty_set(trial_cnt)];
    target.u_set = u_set;
    target.V_set = V_set; % Covariance of process noise model for the target
    target.model_idx = 1;
    target.traj = target.pos;
    target.mode_num = mode_num;
    
    inPara_fld = struct('fld_size',fld_size,'target',target,'tar_move',tar_move,'dt',dt);
    fld = Field(inPara_fld);
            
    % initialize robot class
    
    %%% temp use
    upd_matrix = {eye(fld_size(1)*fld_size(2))};
    inPara_rbt = struct('num_robot',num_robot,'dt',dt);
    rbt = cell(num_robot,1);
    if r_move == 0
        for rbt_cnt = 1:num_robot
            inPara_rbt = struct;
            inPara_rbt.pos = rbt_spec(rbt_cnt).init_pos(:,trial_cnt);
            inPara_rbt.sen_cov = 49*eye(2);%rbt_spec(rbt_cnt).sen_cov;
            inPara_rbt.inv_sen_cov = 0.02*eye(2);%rbt_spec(rbt_cnt).inv_sen_cov;
            inPara_rbt.sen_offset = 0; %rbt_spec(rbt_cnt).sen_offset;
            inPara_rbt.fld_size = fld_size;
            inPara_rbt.max_step = sim_len;
            inPara_rbt.nbhd_idx = rbt_nbhd{rbt_cnt};
            inPara_rbt.r_move = r_move;
            inPara_rbt.num_robot = num_robot;
            inPara_rbt.idx = rbt_cnt;
            inPara_rbt.upd_matrix = upd_matrix;
            rbt{rbt_cnt} = Robot(inPara_rbt);
        end
    elseif r_move == 1
        for rbt_cnt = 1:num_robot
            inPara_rbt = struct;
            inPara_rbt.center = rbt_spec.init_pos(:,rbt_cnt);
            inPara.r = 15;
            inPara_rbt.fld_size = fld_size;
            inPara_rbt.max_step = sim_len;
            inPara_rbt.nbhd_idx = rbt_nbhd{rbt_cnt};
            rbt(rbt_cnt) = Robot(inPara_rbt);
            inPara_pred.u_set = target.u_set;
            inPara_pred.V_set = target.V_set;
            rbt = rbt.predStep(inPara_pred);
        end
    end        
    
    %% %%%%%%%%%%%%%% main code of simulation %%%%%%%%%%%%%%%%%%
    %% LIFO-DBF
    count = 1;
    
    while(1)                
        
        %% filtering
        % Bayesian Updating steps:
        % (1) observe and update the stored own observations at time k
        % (2) exchange and update stored observations
        % (3) update probability map
        % (4) repeat step (1)
        
        % step 1
        % own measurement
        for ii = 1:num_robot
            rbt{ii}.step_cnt = count;            
            % observe
            rbt{ii} = rbt{ii}.sensorGen(fld); % simulate the sensor measurement
            
            % update own observation
            inPara1 = struct('selection',selection);
            rbt{ii} = rbt{ii}.updOwnMsmt(inPara1);
        end        
        
        % step 2
        tmp_rbt = rbt; % use tmp_rbt in data exchange
        % exchange
        for ii = 1:num_robot
            inPara2 = struct;
            inPara2.selection = selection;
            inPara2.rbt_nbhd_set = rbt(rbt{ii}.nbhd_idx);
            tmp_rbt{ii} = tmp_rbt{ii}.dataExch(inPara2);
        end
        rbt = tmp_rbt;
        
        % step 3
        % dbf
        
        %%% temp use
        %%% find the difference of static and moving dbf
        test_rbt = rbt;
        
        for ii = 1:num_robot
            %%% temp use
            testinPara3 = struct('selection',1);
            test_rbt{ii} = test_rbt{ii}.DBF(testinPara3);
            
            inPara3 = struct('selection',selection);
            rbt{ii} = rbt{ii}.DBF(inPara3);
            
            %%% temp use
            tmp_dif = sum(sum(abs(test_rbt{ii}.dbf_map-rbt{ii}.dbf_map)));
            if  tmp_dif >= 10^-14
                display(count)
                display('robot #')
                display(ii)
                display(tmp_dif)               
            end
        end    

        %% draw current step
        % draw plot
        if show_plot
            sim.plotSim(rbt,fld,count);
        end
                
        %% go to next iteration        
        if count > sim_len            
            break
        end
        
        count = count + 1;
        
        % following code should appear at the end of the code. Putting them
        % here is only for debugging purpose
        %% target moves
        if tar_move == 1
            fld = fld.targetMove();
        end
        
        %% robot moves
        if r_move == 1
            for ii = 1:num_robot
                rbt{ii} = rbt{ii}.robotMove();
            end
        end
    end
    
    %% compute metrics
    for ii = 1:num_robot
        rbt{ii} = rbt{ii}.computeMetrics(fld,'dbf');
    end
    
    %% Concensus
    % consider comparing with the Indian guy's NL combination rule.
    %{
    count = 1;
    while(1)
        %% target moves
        fld = fld.targetMove();
        
        %% filtering
        % Bayesian Updating steps:
        % (1) observe and update the stored own observations at time k
        % (2) exchange and update stored observations
        % (3) update probability map
        % (4) repeat step (1)
        for ii = 1:num_robot
            % step 1
            % observe
            rbt{ii} = rbt{ii}.sensorGen(fld); % simulate the sensor measurement
            
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
        
        %% draw current step
        % draw plot
        if show_plot
            sim.plotStim(rbt,fld,trial_cnt);
        end
        
        %% compute metrics
        rbt{ii} = rbt{ii}.computeMetrics();
        %% go to next iteration
        for ii = 1:num_robot
            rbt{ii} = rbt{ii}.stepUpdate;
        end
        
        if count > sim_len            
            break
        end
    end
    %}
    
    
    %% Centralized
    % use rbt{1} to act as the centralized filter
    %{
    count = 1;
    while(1)
        %% target moves
        fld = fld.targetMove();
        
        %% filtering
        % Bayesian Updating steps:
        % (1) observe and update the stored own observations at time k
        % (2) exchange and update stored observations
        % (3) update probability map
        % (4) repeat step (1)
        for ii = 1:num_robot
            % step 1
            % observe
            rbt{ii} = rbt{ii}.sensorGen(fld); % simulate the sensor measurement
            
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
        
        %% draw current step
        % draw plot
        if show_plot
            sim.plotStim(rbt,fld,trial_cnt);
        end
        
        %% compute metrics
        rbt{ii} = rbt{ii}.computeMetrics();
        %% go to next iteration
        for ii = 1:num_robot
            rbt{ii} = rbt{ii}.stepUpdate;
        end
        
        if count > sim_len            
            break
        end
    end
    %}        
    
    sim.rbt_set{trial_cnt} = rbt;
    
    %% target moves
    if tar_move == 1
        fld = fld.targetMove();
    end
    
    %% robot moves
    if r_move == 1
        for ii = 1:num_robot
            rbt{ii} = rbt{ii}.robotMove();
        end
    end
end

%% %%%%%%%%%%%%%%%%%%%%%% Simulation Results %%%%%%%%%%%%%%%%%%%%%%
% compare the performance of different methods
met = sim.compareMetrics();

% save data (workspace)
if save_data 
    
    sim.saveSimData();
end


