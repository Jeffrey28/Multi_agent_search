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
inPara_sim.cons_fig = cons_fig;
inPara_sim.sensor_type = 'brg'; % 'bin': binary,'ran': range-only,'brg': bearing-only,'rb': range-bearing
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
    
    inPara_rbt = struct('num_robot',num_robot,'dt',dt);
    rbt = cell(num_robot,1);
    if r_move == 0
        for rbt_cnt = 1:num_robot
            inPara_rbt = struct;
            inPara_rbt.pos = rbt_spec(rbt_cnt).init_pos(:,trial_cnt);
            inPara_rbt.sen_cov = 100*eye(2);%rbt_spec(rbt_cnt).sen_cov;
            inPara_rbt.inv_sen_cov = 0.01*eye(2);%rbt_spec(rbt_cnt).inv_sen_cov;
            inPara_rbt.sen_offset = 0; %rbt_spec(rbt_cnt).sen_offset;
            inPara_rbt.cov_ran = 1;
            inPara_rbt.dist_ran = 10;
            inPara_rbt.cov_brg = 0.25;
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
            inPara_rbt.center = rbt_spec(rbt_cnt).init_pos(:,rbt_cnt);
            inPara_rbt.r = 5;
            inPara_rbt.T = 20;
            inPara_rbt.w = 2*pi/inPara_rbt.T;
            inPara_rbt.sen_cov = 100*eye(2);%rbt_spec(rbt_cnt).sen_cov;
            inPara_rbt.inv_sen_cov = 0.01*eye(2);%rbt_spec(rbt_cnt).inv_sen_cov;
            inPara_rbt.sen_offset = 0; %rbt_spec(rbt_cnt).sen_offset;
            inPara_rbt.cov_ran = 1;
            inPara_rbt.dist_ran = 10;
            inPara_rbt.cov_brg = 0.25;
            inPara_rbt.fld_size = fld_size;
            inPara_rbt.max_step = sim_len;
            inPara_rbt.nbhd_idx = rbt_nbhd{rbt_cnt};
            inPara_rbt.r_move = r_move;
            inPara_rbt.num_robot = num_robot;
            inPara_rbt.idx = rbt_cnt;
            inPara_rbt.upd_matrix = upd_matrix;
            rbt{rbt_cnt} = Robot(inPara_rbt);
%             inPara_pred.u_set = target.u_set;
%             inPara_pred.V_set = target.V_set;
%             rbt = rbt.predStep(inPara_pred);
        end
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
                 rbt{ii} = rbt{ii}.robotMove();
             end
         end
        
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
            
            switch sim.sensor_type
                case 'bin'
                    rbt{ii} = rbt{ii}.sensorGenBin(fld); % simulate the sensor measurement
                case 'ran'
                    rbt{ii} = rbt{ii}.sensorGenRan(fld);
                case 'brg'
                    rbt{ii} = rbt{ii}.sensorGenBrg(fld);
                case 'rb'
                    rbt{ii} = rbt{ii}.sensorGenRb(fld);
            end
                        
            % update own observation                      
            inPara1 = struct('selection',selection);
            rbt{ii} = rbt{ii}.updOwnMsmt(inPara1);
        end        
        
        %% LIFO-DBF
        % step 2       
        % exchange
        tmp_rbt = rbt; % use tmp_rbt in data exchange
        
        for ii = 1:num_robot
            inPara2 = struct;
            inPara2.selection = selection;
            inPara2.rbt_nbhd_set = rbt(rbt{ii}.nbhd_idx);
            tmp_rbt{ii} = tmp_rbt{ii}.dataExch(inPara2);
        end
        rbt = tmp_rbt;
        
        % step 3
        % dbf
        for ii = 1:num_robot                        
            inPara3 = struct('selection',selection);
            rbt{ii} = rbt{ii}.DBF(inPara3);                        
        end    

         %% Concensus        
         %
         % update own map using own measurement
         for ii = 1:num_robot  
             inPara4 = struct('selection',selection);
             rbt{ii} = rbt{ii}.updMap(inPara4);
         end
         
         % exchange with neighbors to achieve consensus
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
         %
         % use the first robot as the centralized filter
         rbt{1}.buffer_cent.pos = rbt{1}.pos;
         rbt{1}.buffer_cent.z = rbt{1}.z;
         rbt{1}.buffer_cent.k = rbt{1}.k;
         rbt{1}.buffer_cent.lkhd_map = {rbt{1}.lkhd_map};
         
         for ii = 2:num_robot
             rbt{1}.buffer_cent.pos = [rbt{1}.buffer_cent.pos,rbt{ii}.pos];
             rbt{1}.buffer_cent.z = [rbt{1}.buffer_cent.z,rbt{ii}.z];
             rbt{1}.buffer_cent.k = [rbt{1}.buffer_cent.k,rbt{ii}.k];
             rbt{1}.buffer_cent.lkhd_map = [rbt{1}.buffer_cent.lkhd_map,rbt{ii}.lkhd_map];
         end
         inPara6 = struct;
         inPara6.selection = selection;
         
         rbt{1} = rbt{1}.CF(inPara6);
         %}
         
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
         
         %% compute metrics
         for ii = 1:num_robot
             rbt{ii} = rbt{ii}.computeMetrics(fld,'dbf');
             rbt{ii} = rbt{ii}.computeMetrics(fld,'cons');
             rbt{ii} = rbt{ii}.computeMetrics(fld,'cent');
         end
    end
        
    sim.rbt_set{trial_cnt}.rbt = rbt;
    sim.fld_set{trial_cnt}.fld = fld;
    
%     %% target moves
%     if tar_move == 1
%         fld = fld.targetMove();
%     end
%     
%     %% robot moves
%     if r_move == 1
%         for ii = 1:num_robot
%             rbt{ii} = rbt{ii}.robotMove();
%         end
%     end
end

%% %%%%%%%%%%%%%%%%%%%%%% Simulation Results %%%%%%%%%%%%%%%%%%%%%%
% % compare the performance of different methods
met = sim.compareMetrics();
% 
% % save data (workspace)
% if save_data
%     file_name = sim.saveSimData();
%     save(file_name,'sim','fld')
% end