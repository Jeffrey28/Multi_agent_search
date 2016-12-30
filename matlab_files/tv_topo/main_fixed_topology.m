%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Multi Sensor-based Distributed Bayesian Estimator
% This code is for distributed bayesian estimator for target positioning and tracking
% (1) Target: Static or moving target with unknown position
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
% 7/23/16
% continued re-writing the code in OOP. the paper based on this code got
% rejected in CDC 16
% 10/3/16
% continued debugging the code. will implement different sensor model
% 10/14/16
% this file is for generating and analyzing simulation data

% main function for running the simulation. use with simSetup
clearvars -except upd_matrix
clc; close all

%% %%%%%%% Simulation %%%%%%%%%%
simSetup();

% define parameters, precompute certain quantities

% initialize sim class
inPara = struct;
inPara_sim.r_move = r_move;
inPara_sim.tar_move = tar_move;
inPara_sim.sim_len = sim_len;
inPara_sim.cons_step = cons_step;
inPara_sim.num_robot = num_robot;
inPara_sim.r_init_pos_set = extractfield(rbt_spec,'init_pos');
inPara_sim.fld_size = fld_size;
inPara_sim.t_init_pos_set = [tx_set;ty_set];
inPara_sim.sim_r_idx = sim_r_idx;
inPara_sim.trial_num = trial_num;
inPara_sim.dt = dt;
inPara_sim.selection = selection;
inPara_sim.cons_fig = cons_fig;
inPara_sim.sensor_set_type = sensor_set_type; % 'bin': binary,'ran': range-only,'brg': bearing-only,'rb': range-bearing
sim = Sim(inPara_sim);

for trial_cnt = 1%1:trial_num
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
    
    inPara_rbt = struct('num_robot',num_robot,'dt',dt);
    rbt = cell(num_robot,1);
    
    for rbt_cnt = 1:num_robot
        inPara_rbt = struct;
        if r_move == 0
            inPara_rbt.pos = rbt_spec(rbt_cnt).init_pos(:,trial_cnt);
        elseif r_move == 1
            inPara_rbt.center = rbt_spec(rbt_cnt).init_pos(:,rbt_cnt);
            inPara_rbt.r = r_set(rbt_cnt);
            inPara_rbt.T = T_set(rbt_cnt);
            inPara_rbt.w = dir_set(rbt_cnt)*2*pi/inPara_rbt.T;
        elseif r_move == 2
            inPara_rbt.pos = r_init_pos_exp(rbt_cnt);
        end
        
        % binary sensor
%         inPara_rbt.sen_cov = 100*eye(2);
%         inPara_rbt.inv_sen_cov = 0.01*eye(2);
%         inPara_rbt.sen_offset = 0;
        % range-only sensor
        inPara_rbt.cov_ran = 5;%5
        inPara_rbt.dist_ran = 50;
        inPara_rbt.offset_ran = 0;
        % bearing-only sensor
        inPara_rbt.cov_brg = 0.5;%0.5
        inPara_rbt.offset_brg = 0;
        % range-bearing sensor
        inPara_rbt.dist_ranbrg = 20;
        inPara_rbt.cov_ranbrg = 1*eye(2);%100
        
        inPara_rbt.fld_size = fld_size;
        inPara_rbt.max_step = sim_len;
        inPara_rbt.nbhd_idx = rbt_nbhd{rbt_cnt};
        inPara_rbt.r_move = r_move;
        inPara_rbt.num_robot = num_robot;
        inPara_rbt.idx = rbt_cnt;
        inPara_rbt.upd_matrix = upd_matrix;
        inPara_rbt.sensor_type = sensor_set{rbt_cnt};
        rbt{rbt_cnt} = Robot(inPara_rbt);
    end    
    %% %%%%%%%%%%%%%% main code of simulation %%%%%%%%%%%%%%%%%%
    count = 1;
    
    while(1)                        
        % following code should appear at the end of the code. Putting them
        % here is only for debugging purpose
        %% target moves
        if tar_move == 1
            fld = fld.targetMove();
        end
        
        %% robot moves
        if r_move == 1
            for ii = 1:num_robot
                rbt{ii}.tar_mod = [rbt{ii}.tar_mod,fld.target.model_idx];
                rbt{ii} = rbt{ii}.robotMove();
            end
        elseif r_move == 2
            for ii = 1:num_robot
                rbt{ii} = r_pos_set_exp(ii,k);
            end
        end
        
        %% filtering
        % generate sensor measurement
        for ii = 1:num_robot
            rbt{ii}.step_cnt = count;
            % observe
            if sim_mode
                switch rbt{ii}.sensor_type
                    case 'bin'
                        rbt{ii} = rbt{ii}.sensorGenBin(fld); % simulate the sensor measurement
                    case 'ran'
                        rbt{ii} = rbt{ii}.sensorGenRan(fld);
                    case 'brg'
                        rbt{ii} = rbt{ii}.sensorGenBrg(fld);
                    case 'rb'
                        rbt{ii} = rbt{ii}.sensorGenRanBrg(fld);
                end
            elseif exp_mode
                rbt{ii} = rbt{ii}.sensorGenRan(fld);
            end
                        
            % update own observation                      
            inPara1 = struct('selection',selection);
            rbt{ii} = rbt{ii}.updOwnMsmt(inPara1);
        end        
        
        %% LIFO-DBF
        % (1) exchange stored observations
        % (2) observe and update the stored own observations at time k        
        % (3) update probability map
        % (4) repeat step (1)
        % so measurement from other sensors are one-step behind
        
        % note, step (2) is in previous code section (update own measurement)
        % so continue to step (3)
        
        % step (3) dbf
        for ii = 1:num_robot                        
            inPara3 = struct('selection',selection,'target_model',fld.target.model_idx);
            rbt{ii} = rbt{ii}.DBF(inPara3);                        
        end    

        % step (1) exchange
        tmp_rbt = rbt; % use tmp_rbt in data exchange
        
        for ii = 1:num_robot
            inPara2 = struct;
            inPara2.selection = selection;
            inPara2.rbt_nbhd_set = rbt(rbt{ii}.nbhd_idx);
            tmp_rbt{ii} = tmp_rbt{ii}.dataExch(inPara2);
        end
        rbt = tmp_rbt;
        
         %% Concensus        
         % (1) exchange pdfs to achieve concensus
         % (2) observe and update the stored own observations at time k
         % (3) update probability map
         % (4) repeat step (1)
         % so consensus actually uses more information than DBF at each
         % step when doing metric comparison 
         
         % step (3) update own map using own measurement
         for ii = 1:num_robot  
             inPara4 = struct('selection',selection,'target_model',fld.target.model_idx);
             rbt{ii} = rbt{ii}.updMap(inPara4);
         end
         
         % step (1) exchange with neighbors to achieve consensus
         % consider comparing with the Indian guy's NL combination rule.         
         cons_cnt = 1;
         while (cons_cnt <= cons_step)
             tmp_rbt_cons = rbt; % use tmp_rbt_cons for consensus
             for ii = 1:num_robot
                 inPara5 = struct;
                 inPara5.selection = selection;
                 inPara5.cons_fig = cons_fig;
                 inPara5.rbt_nbhd_set = rbt(rbt{ii}.nbhd_idx);
                 tmp_rbt_cons{ii} = tmp_rbt_cons{ii}.cons(inPara5);
             end
             rbt = tmp_rbt_cons;
             cons_cnt = cons_cnt + 1;
         end         
         %}
        
         %% centeralized filter
         % (1) observe and update the stored observations of all sensor at time k
         % (2) update probability map
         % (2) repeat step (1)         
         
         % use the first robot as the centralized filter
         rbt{1}.buffer_cent.pos = rbt{1}.pos;
         rbt{1}.buffer_cent.z = {rbt{1}.z};
         rbt{1}.buffer_cent.k = rbt{1}.k;
         rbt{1}.buffer_cent.lkhd_map = {rbt{1}.lkhd_map};
         
         for ii = 2:num_robot
             rbt{1}.buffer_cent.pos = [rbt{1}.buffer_cent.pos,rbt{ii}.pos];
             rbt{1}.buffer_cent.z = [rbt{1}.buffer_cent.z,{rbt{ii}.z}];
             rbt{1}.buffer_cent.k = [rbt{1}.buffer_cent.k,rbt{ii}.k];
             rbt{1}.buffer_cent.lkhd_map = [rbt{1}.buffer_cent.lkhd_map,rbt{ii}.lkhd_map];
         end
         inPara6 = struct;
         inPara6.selection = selection;
         inPara6.target_model = fld.target.model_idx;
         
         rbt{1} = rbt{1}.CF(inPara6);
         %}
         
         %% draw current step
         % draw plot
         if show_plot
             sim.plotSim(rbt,fld,count,save_plot);
%              pause()
         end
                
         %% go to next iteration
         if count > sim_len
             break
         end
         
         count = count + 1;
         
         %% compute metrics
         for ii = 1:num_robot
             rbt{ii} = rbt{ii}.computeMetrics(fld,'dbf');
             rbt{ii} = rbt{ii}.computeMetrics(fld,'cons');
             rbt{ii} = rbt{ii}.computeMetrics(fld,'cent');
         end
    end
        
    sim.rbt_set{trial_cnt}.rbt = rbt;
    sim.fld_set{trial_cnt}.fld = fld;   
end

%% %%%%%%%%%%%%%%%%%%%%%% Simulation Results %%%%%%%%%%%%%%%%%%%%%%
% % compare the performance of different methods
sim = sim.compareMetrics();

% save data 
% note 'sim' contains all data about rbt, fld and simulation results.
% However it is so huge that we cannot save all...
if save_data
    sim_for_save = sim;
    sim_for_save.rbt_set = {};
    sim_for_save.fld_set = {};
    file_name = sim.saveSimData();
    save(file_name,'sim_for_save','-v7')
end