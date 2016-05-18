classdef Robot
    properties
        idx; % index of the robot
        pos; % robot position
        upd_cell; % cell used for updating probability map
        step_cnt; % current time step
        % used for circular motion of robot
        T; % period of circular motion
        r; % radius
        w; % angular velocity
        center; % center of the circular motion
        % sensor specs
        sen_cov;
        inv_sen_cov;
        sen_offset;
        % observation
        z; % observation measurement
        k; % measurement time
        prob; % prob matrix for a certain observation. a way to reduce computation time, sacrificing the space complexity
        
        traj;
        talign_map; % store the prob_map for observations with same tiem index, i.e. P(x|z^1_1:k,...,z^N_1:k)
        talign_t; % record the time for the time-aligned map
        dbf_map; % probability map
        cons_map; % prob map for concensus method
        cen_map; % prob map for centralized filter
        entropy;
        nbhd_idx; % index of neighboring robots
        nbhd_record; % record some useful info about neighbors, e.g. the time steps that have been used for updating the prob map
        num_robot;
        color; % color for drawing plots
        buffer; % communication buffer. a struct array, each element corresponding to the latest info for a robot
        % performance metrics
        ml_pos_dbf;
        ml_pos_cons;
        ml_pos_cent;
        ml_err_dbf;
        ml_err_cons;
        ml_err_cent;
        pdf_cov_dbf;
        pdf_cov_cons;
        pdf_cov_cent;
        pdf_norm_dbf;
        pdf_norm_cons;
        pdf_norm_cent;
        ent_dbf;
        ent_cons;
        ent_cent;
    end
    
    methods
        function obj = Robot(inPara)
            obj.pos = inPara.pos;
            obj.upd_cell = {}; %%% can precompute this term and pass the lookup table to this constructor function
            
            if inPara.r_move == 0
                obj.pos = inPara.pos; % sensor position
            elseif inPara.r_move == 1
                obj.T = 20; % period of circling motion
                obj.center = inPara.center;
                obj.r = inPara.r; %15;
                obj.w = 2*pi/obj.T;
                obj.pos = [inPara.center(1);inPara.center(1)+obj.r]; % initial position is at the top of the circle
            end
            obj.traj = [];
            obj.sen_cov = inPara.sen_cov;
            obj.inv_sen_cov = inPara.inv_sen_cov;
            obj.sen_offset = inPara.sen_offset;
            obj.dbf_map = ones(inPara.fld_size(1),inPara.fld_size(2));
            obj.dbf_map = obj.dbf_map/sum(sum(obj.dbf_map));
            obj.cons_map = obj.dbf_map;
            obj.cen_map = obj.dbf_map;
            obj.talign_map = obj.dbf_map; % store the prob_map for observations with same tiem index, i.e. P(x|z^1_1:k,...,z^N_1:k)
            obj.talign_t = 0; % record the time for the time-aligned map
            obj.prob = zeros(inPara.fld_size(1),inPara.fld_size(2));
            obj.entropy = zeros(1,inPara.max_step);
            obj.nbhd_idx = inPara.nbhd_idx;    
            % initialize buffer
            obj.buffer(inPara.num_robot).pos = [];
            obj.buffer(inPara.num_robot).z = [];
            obj.buffer(inPara.num_robot).k = [];
            obj.buffer(inPara.num_robot).prob = {};
            obj.buffer(inPara.num_robot).used = [];
%             obj.buffer = struct;
%             if ~isempty(inPara.nbhd_idx)
%                 obj.buffer(inPara.num_robot).used = []; % record the observations that are already used by BF
%             end
            obj.num_robot = inPara.num_robot;
            obj.step_cnt = 0;
        end
        
        function sensorGen(this,fld)
            x_r = this.pos;
            t = fld.target.pos;
            inv_cov = this.inv_sen_cov;
            offset = this.sen_offset;
            
            prob = exp(-1/2*(t+offset-x_r)'*inv_cov*(t+offset-x_r));
            z = (rand(1,1) < prob);
            if z == 0
                prob = 1-prob;
            end                
            this.z = z;
            this.k = this.step_cnt;
            this.prob = prob;
        end
        
        function rbt = updOwnMsmt(rbt,inPara)
            % update robot's own measurement in the communication buffer
            
            selection = inPara.selection;
            if (selection == 1) || (selection == 3)
                % static target
                % (1) self-observation
                % update the buffer with the robot's own observation
                rbt.buffer(rbt.idx).pos = rbt.pos;
                rbt.buffer(rbt.idx).z = rbt.z;
                rbt.buffer(rbt.idx).k = rbt.step_cnt;
                rbt.buffer(rbt.idx).prob = rbt.prob;
                
%                 if (~isempty(rbt.buffer(rbt.idx).k)) && (rbt.buffer(rbt.idx).z == 1)
%                     rbt.buffer(rbt.idx).prob = rbt.sensorProb(inPara.fld);
                    
%                 elseif (~isempty(rbt.buffer(rbt.idx).k)) && (rbt.buffer(rbt.idx).z == 0)
%                     rbt.buffer(rbt.idx).prob = 1 - rbt.sensorProb(inPara.fld);
%                 end
                
                % assign this probability to rbt_cons and rbt_cent to
                % save computation resource
%                 rbt.cons_prob = rbt.buffer(rbt.idx).prob;
%                 rbt.cent_prob = rbt.buffer(rbt.idx).prob;
                %                 rbt_cons(i).prob = rbtBuffer{i}.rbt(i).prob;
                %                 rbt_cent.prob{i} = rbtBuffer{i}.rbt(i).prob;
                
                %                 display(rbtBuffer{1}.rbt(1)),display(rbtBuffer{1}.rbt(2)),display(rbtBuffer{1}.rbt(3))
                %                 display(rbtBuffer{1}.rbt(4)),display(rbtBuffer{1}.rbt(5)),display(rbtBuffer{1}.rbt(6))
            elseif (selection == 2) || (selection == 4)
                % Observation of each robot
                %                 for i=1:NumOfRobot
                rbt.buffer(rbt.idx).pos = [rbt.pos,rbt.buffer(rbt.idx).pos];
                rbt.buffer(rbt.idx).z = [rbt.z,rbt.buffer(rbt.idx).z];
                rbt.buffer(rbt.idx).k = [rbt.k,rbt.buffer(rbt.idx).k];
                rbt.buffer(rbt.idx).prob{rbt.step_cnt} = rbt.prob;

%                 if (~isempty(rbt.buffer(rbt.idx).k)) && (rbt.buffer(rbt.idx).z == 1)
%                     rbt.buffer(rbt.idx).prob = rbt.sensorProb(inPara.fld);
%                 elseif (~isempty(rbt.buffer(rbt.idx).k)) && (rbt.buffer(rbt.idx).z == 0)
%                     rbt.buffer(rbt.idx).prob = 1 - rbt.sensorProb(inPara.fld);
%                 end
                
                % assign this probability to rbt_cons and rbt_cent to
                % save computation resource
%                 rbt.cons_prob = rbt.buffer(rbt.idx).prob;
%                 rbt.cent_prob = rbt.buffer(rbt.idx).prob;
                
                %                     rbtBuffer{i}.rbt(i).x=[rbt(i).x,rbtBuffer{i}.rbt(i).x];
                %                     rbtBuffer{i}.rbt(i).y=[rbt(i).y,rbtBuffer{i}.rbt(i).y];
                %                     rbtBuffer{i}.rbt(i).z=[rbt(i).z,rbtBuffer{i}.rbt(i).z];
                %                     rbtBuffer{i}.rbt(i).k=[count,rbtBuffer{i}.rbt(i).k];
                % remove unneeded records
                % only observations that are got no less than talign_t+1
                % are used for Bayes filtering
                %                     rmv_idx = rbtBuffer{i}.rbt(i).k <= rbt(i).talign_t;
                %                     rbtBuffer{i}.rbt(i).x(rmv_idx)=[];
                %                     rbtBuffer{i}.rbt(i).y(rmv_idx)=[];
                %                     rbtBuffer{i}.rbt(i).z(rmv_idx)=[];
                %                     rbtBuffer{i}.rbt(i).k(rmv_idx)=[];
                %                 end
                
                %                 display(rbtBuffer{1}.rbt(1)),display(rbtBuffer{1}.rbt(2)),display(rbtBuffer{1}.rbt(3))
                %                 display(rbtBuffer{1}.rbt(4)),display(rbtBuffer{1}.rbt(5)),display(rbtBuffer{1}.rbt(6))
            end
        end
        
        function rbt = dataExch(rbt,inPara)
            % exchange communication buffer with neighbors
            selection = inPara.selection;
            rbt_nbhd_set = inPara.rbt_nbhd_set;
            if (selection == 1) || (selection == 3)
                % (2) sending/receive
                for t = 1:length(rbt_nbhd_set)
                    % note: communication only transmit the latest
                    % observation stored in each neighbor
                    tmp_rbt = rbt_nbhd_set(t);
                    for jj = 1:rbt.num_robot
                        if (~isempty(tmp_rbt.buffer(jj).k)) && (isempty(rbt.buffer(jj).k) || (rbt.buffer(jj).k < tmp_rbt.buffer(jj).k))
                            rbt.buffer(jj).pos = tmp_rbt.buffer(jj).pos;
                            rbt.buffer(jj).z = tmp_rbt.buffer(jj).z;
                            rbt.buffer(jj).k = tmp_rbt.buffer(jj).k;
                            rbt.buffer(jj).prob = tmp_rbt.buffer(jj).prob;
                        end
                    end
                    
                    %                             if (~isempty(rbtBuffer{t}.rbt(j).k)) && (isempty(tempRbtBuffer{i}.rbt(j).k) || (tempRbtBuffer{i}.rbt(j).k < rbtBuffer{t}.rbt(j).k))
                    %                                 tempRbtBuffer{i}.rbt(j).x = rbtBuffer{t}.rbt(j).x;
                    %                                 tempRbtBuffer{i}.rbt(j).y = rbtBuffer{t}.rbt(j).y;
                    %                                 tempRbtBuffer{i}.rbt(j).z = rbtBuffer{t}.rbt(j).z;
                    %                                 tempRbtBuffer{i}.rbt(j).k = rbtBuffer{t}.rbt(j).k;
                    %                                 tempRbtBuffer{i}.rbt(j).prob = rbtBuffer{t}.rbt(j).prob;
                    %                             end
                end
                %                     end
                %                 end
                
                % return temperary buffer to robot buffer
                %                 for i=1:NumOfRobot
                %                     for j=1:NumOfRobot
                %                         rbtBuffer{i}.rbt(j).x = tempRbtBuffer{i}.rbt(j).x;
                %                         rbtBuffer{i}.rbt(j).y = tempRbtBuffer{i}.rbt(j).y;
                %                         rbtBuffer{i}.rbt(j).z = tempRbtBuffer{i}.rbt(j).z;
                %                         rbtBuffer{i}.rbt(j).k = tempRbtBuffer{i}.rbt(j).k;
                %                         rbtBuffer{i}.rbt(j).prob = tempRbtBuffer{i}.rbt(j).prob;
                %                     end
                %                 end
                
            elseif (selection == 2) || (selection == 4)
                % moving target
                %% data transmission
                %                 % (1) observation
                %                 % Observation of each robot
                %                 for i=1:NumOfRobot
                %                     rbtBuffer{i}.rbt(i).x=[rbt(i).x,rbtBuffer{i}.rbt(i).x];
                %                     rbtBuffer{i}.rbt(i).y=[rbt(i).y,rbtBuffer{i}.rbt(i).y];
                %                     rbtBuffer{i}.rbt(i).z=[rbt(i).z,rbtBuffer{i}.rbt(i).z];
                %                     rbtBuffer{i}.rbt(i).k=[count,rbtBuffer{i}.rbt(i).k];
                %                     % remove unneeded records
                %                     % only observations that are got no less than talign_t+1
                %                     % are used for Bayes filtering
                %                     rmv_idx = rbtBuffer{i}.rbt(i).k <= rbt(i).talign_t;
                %                     rbtBuffer{i}.rbt(i).x(rmv_idx)=[];
                %                     rbtBuffer{i}.rbt(i).y(rmv_idx)=[];
                %                     rbtBuffer{i}.rbt(i).z(rmv_idx)=[];
                %                     rbtBuffer{i}.rbt(i).k(rmv_idx)=[];
                %                 end
                %
                %                 display(rbtBuffer{1}.rbt(1)),display(rbtBuffer{1}.rbt(2)),display(rbtBuffer{1}.rbt(3))
                %                 display(rbtBuffer{1}.rbt(4)),display(rbtBuffer{1}.rbt(5)),display(rbtBuffer{1}.rbt(6))
                
                % (2) sending/receive
                % multi-step transmit of observation
                for t = 1:length(rbt_nbhd_set)
                    % note: communication only transmit the latest
                    % observation stored in each neighbor
                    tmp_rbt = rbt_nbhd_set(t);
                    for jj = 1:rbt.num_robot
                        if (~isempty(tmp_rbt.buffer(jj).k)) && (isempty(rbt.buffer(jj).k) || (rbt.buffer(jj).k < tmp_rbt.buffer(jj).k))
                            %%% this code only handles the fixed topology
                            %%% case, i.e. each time a one-step newer
                            %%% observation is received. for multi-step
                            %%% newer observations, such code needs
                            %%% modification.
                            rbt.buffer(jj).pos = [tmp_rbt.buffer(jj).pos(:,1),rbt.buffer(jj).pos];
                            rbt.buffer(jj).z = [tmp_rbt.buffer(jj).z(1),rbt.buffer(jj).z];
                            rbt.buffer(jj).k = [tmp_rbt.buffer(jj).k(1),rbt.buffer(jj).k];
                            % in real experiment, this prob term should not
                            % be communicated. In simulation, this is for
                            % the purpose of accelerating the computation speed.
                            rbt.buffer(jj).prob = {tmp_rbt.buffer(jj).prob{1},rbt.buffer(jj).prob};
                        end
                    end
                end
                
                %                 tempRbtBuffer=rbtBuffer;
                %                 for ii=1:rbt.num_robot % Robot Iteration
                %                     % for information from neighbours to compare whether it is
                %                     % latest
                %                     for jj=1:NumOfRobot
                %                         for t=rbt(ii).neighbour
                %                             % note: communication only transmit the latest
                %                             % observation stored in each neighbor
                %                             if (~isempty(rbtBuffer{t}.rbt(jj).k)) && (isempty(tempRbtBuffer{ii}.rbt(jj).k) || (tempRbtBuffer{ii}.rbt(jj).k(1) < rbtBuffer{t}.rbt(jj).k(1)))
                %                                 tempRbtBuffer{ii}.rbt(jj).x = [rbtBuffer{t}.rbt(jj).x(1),tempRbtBuffer{ii}.rbt(jj).x];
                %                                 tempRbtBuffer{ii}.rbt(jj).y = [rbtBuffer{t}.rbt(jj).y(1),tempRbtBuffer{ii}.rbt(jj).y];
                %                                 tempRbtBuffer{ii}.rbt(jj).z = [rbtBuffer{t}.rbt(jj).z(1),tempRbtBuffer{ii}.rbt(jj).z];
                %                                 tempRbtBuffer{ii}.rbt(jj).k = [rbtBuffer{t}.rbt(jj).k(1),tempRbtBuffer{ii}.rbt(jj).k];
                %                             end
                %                         end
                %                         % remove unneeded records
                %                         rmv_idx = tempRbtBuffer{ii}.rbt(jj).k <= rbt(ii).talign_t;
                %                         tempRbtBuffer{ii}.rbt(jj).x(rmv_idx)=[];
                %                         tempRbtBuffer{ii}.rbt(jj).y(rmv_idx)=[];
                %                         tempRbtBuffer{ii}.rbt(jj).z(rmv_idx)=[];
                %                         tempRbtBuffer{ii}.rbt(jj).k(rmv_idx)=[];
                %                     end
                %                 end
                %
                %                 % return temperary buffer to robot buffer
                %                 for ii=1:NumOfRobot
                %                     for jj=1:NumOfRobot
                %                         rbtBuffer{ii}.rbt(jj).x = tempRbtBuffer{ii}.rbt(jj).x;
                %                         rbtBuffer{ii}.rbt(jj).y = tempRbtBuffer{ii}.rbt(jj).y;
                %                         rbtBuffer{ii}.rbt(jj).z = tempRbtBuffer{ii}.rbt(jj).z;
                %                         rbtBuffer{ii}.rbt(jj).k = tempRbtBuffer{ii}.rbt(jj).k;
                %                     end
                %                 end
            end
        end
        
        function rbt = DBF(rbt,inPara)
            selection = inPara.selection;
            step_cnt = rbt.step_cnt;
            upd_cell = rbt.upd_cell;
            if (selection == 1) || (selection == 3)
                %% update by bayes rule
                % calculate probility of latest z
                for jj=1:rbt.num_robot % Robot Iteration
                    if (~isempty(rbt.buffer(jj).k)) && (~ismember(rbt.buffer(jj).k,rbt.buffer(jj).used))
                        rbt.dbf_map=rbt.dbf_map.*rbt.buffer(jj).prob;
                        rbt.buffer(jj).used = [rbt.buffer(jj).used,rbt.buffer(jj).k];
                    end
                end
                rbt.dbf_map=rbt.dbf_map/sum(sum(rbt.dbf_map));
                
            elseif (selection == 2) || (selection == 4)
                %% update by bayes rule
                % note: main computation resource are used in calling sensorProb function.
                % when using grid map, can consider precomputing
                % results and save as a lookup table
                
                %                 for ii=1:NumOfRobot % Robot Iteration
                talign_flag = 1; % if all agent's observation's time are no less than talign_t+1, then talign_flag = 1, increase talign_t
                tmp_t = rbt.talign_t;
                tmp_map = rbt.talign_map; % time-aligned map
                
                for t = (rbt.talign_t+1):step_cnt
                    %% one-step prediction step
                    tmp_map2 = zeros(size(tmp_map));
                    for k = 1:size(pt,1)
                        tmp_map2 = tmp_map2+upd_cell{k,model_idx}*tmp_map(pt(k,1),pt(k,2));
                    end
                    tmp_map = tmp_map2;
                    
                    %% updating step
                    %                         for jj=1:rbt.num_robot
                    %                             if (~isempty(rbt.buffer(jj).k)) && (rbt.buffer(jj).k(1) >= t)
                    %                                 if t < rbt.buffer(jj).k(1)
                    %                                     rbt.buffer(jj).prob{t} = rbtBuffer{ii}.rbt(jj).map{t};
                    %                                 elseif t == rbtBuffer{ii}.rbt(jj).k(1)
                    %                                     if rbtBuffer{ii}.rbt(jj).z(1) == 1
                    %                                         rbtBuffer{ii}.rbt(jj).prob = sensorProb(rbtBuffer{ii}.rbt(jj).x(1),rbtBuffer{ii}.rbt(jj).y(1),fld.x,fld.y,sigmaVal);
                    %                                     elseif rbtBuffer{ii}.rbt(jj).z(1) == 0
                    %                                         rbtBuffer{ii}.rbt(jj).prob = 1 - sensorProb(rbtBuffer{ii}.rbt(jj).x(1),rbtBuffer{ii}.rbt(jj).y(1),fld.x,fld.y,sigmaVal);
                    %                                     end
                    %                                     % record the previously calculated map to
                    %                                     % reduce computation
                    %                                     rbtBuffer{ii}.rbt(jj).map{t} = rbtBuffer{ii}.rbt(jj).prob;
                    %                                 end
                    %                                 tmp_map = tmp_map.*rbtBuffer{ii}.rbt(jj).prob;
                    %                             else
                    %                                 talign_flag = 0;
                    %                             end
                    %                         end
                    
                    for jj=1:rbt.num_robot
                        if (~isempty(rbt.buffer(jj).k)) && (rbt.buffer(jj).k(1) >= t)
                            % note: this update is not valid in real
                            % experiment since we don't communicate
                            % probability. This line of code is for
                            % computation reduction in simulation
                            tmp_map = tmp_map.*rbt.buffer(jj).prob{t};
                        else
                            talign_flag = 0;
                        end
                    end
                    
                    % assign the probability to rbt_cons and rbt_cent to
                    % save computation resource
                    %                         rbt_cons(ii).prob = rbtBuffer{ii}.rbt(ii).prob;
                    %                         rbt_cent.prob{ii} = rbtBuffer{ii}.rbt(ii).prob;
                    
                    % after the first loop, the robot's aligned time
                    % increase by one
                    if (t == rbt.talign_t+1) && (talign_flag == 1)
                        rbt.talign_map = tmp_map;
                        rbt.talign_map = rbt.talign_map/sum(sum(rbt.talign_map));
                        tmp_t = tmp_t+1;
                    end
                end
                
                rbt(ii).talign_t = tmp_t;
                rbt(ii).map = tmp_map;
                rbt(ii).map = rbt(ii).map/sum(sum(rbt(ii).map));
                %                 end
                % record the map for each time
                %             rbt(i).map_cell{count} = rbt(i).map;
            end
        end
        
        function rbt = cons(rbt,inPara)
            %% %%%%%%%%%%%%%%  Consensus Method %%%%%%%%%%%%%%%%%%
            % steps:
            % (1) observe and update the probability map for time k
            % (2) send/receive the probability map for time k-1 from neighbors
            % (3) repeat step (1)
            % note: the steps are opposite to the DBF steps, which first exchange
            % info and then incorporate new observations. I still need to think
            % carefully which order is more reasonable. But for now, I think it's
            % better to present the consensused results in paper so that readers
            % will not get confused.
            
            % update using new observation
            if (Selection == 1) || (Selection == 3)
                % update probability map
                for i=1:NumOfRobot
                    tmp_cons_map = rbt_cons(i).map.*rbt_cons(i).prob;
                    rbt_cons(i).map = tmp_cons_map/sum(sum(tmp_cons_map));
                end
                %}
            elseif (Selection == 2) || (Selection == 4)
                for i=1:NumOfRobot
                    tmp_cons_map = rbt_cons(i).map;
                    % prediction step
                    tmp_map_cons2 = zeros(size(tmp_cons_map));
                    for k = 1:size(pt,1)
                        tmp_map_cons2 = tmp_map_cons2+upd_cell1{k,model_idx}*tmp_cons_map(pt(k,1),pt(k,2));
                    end
                    tmp_cons_map = tmp_map_cons2;
                    
                    % update step
                    tmp_cons_map = tmp_cons_map.*rbt_cons(i).prob;
                    rbt_cons(i).map = tmp_cons_map/sum(sum(tmp_cons_map));
                end
            end
            
            % consensus step
            % receive and weighted average neighboring maps
            for i=1:NumOfRobot % Robot Iteration
                rbtCon(i).map=rbt_cons(i).map;
            end
            
            for ConStep=1:ConsenStep % Consensus cycle
                if ConsenFigure==1
                    fig_cnt = fig_cnt+1;
                    h_cons = figure(fig_cnt);
                    clf(h_cons);
                end
                for i=1:NumOfRobot % Robot Iteration
                    neighNum=length(rbt(i).neighbour)+1;
                    tempRbtCon(i).map = rbtCon(i).map;
                    for t=rbt(i).neighbour
                        tempRbtCon(i).map=tempRbtCon(i).map+rbtCon(t).map;
                    end
                    tempRbtCon(i).map=tempRbtCon(i).map/neighNum;
                end
                % plot local PDFs after concensus
                for i=1:NumOfRobot
                    rbtCon(i).map=tempRbtCon(i).map;
                    if ConsenFigure==1
                        figure(fig_cnt)
                        subplot(2,3,i); contourf((rbtCon(i).map)'); title(['Sensor ',num2str(i)]);
                        hold on;
                        for j=1:NumOfRobot
                            if i==j
                                plot(rbt(j).x, rbt(j).y, 's','Color',rbt(j).color,'MarkerSize',8,'LineWidth',3);
                            else
                                plot(rbt(j).x, rbt(j).y, 'p','Color',rbt(j).color, 'MarkerSize',8,'LineWidth',1.5);
                            end
                        end
                    end
                end
            end
            for i=1:NumOfRobot % Robot Iteration
                rbt_cons(i).map=rbtCon(i).map;
            end
        end
        
        function rbt = CF(rbt,inPara)
            %% %%%%%%%%%%%%%% Centralized BF %%%%%%%%%%%%%%%%%%
            % steps:
            % (1) receive all robots' observations
            % (2) update the probability map for time k
            % (3) repeat step (1)
            
            tmp_cent_map = rbt_cent.map;
            if (Selection == 1) || (Selection == 3)
                % update step
                for i = 1:NumOfRobot
                    tmp_cent_map = tmp_cent_map.*rbt_cent.prob{i};
                end
                rbt_cent.map = tmp_cent_map/sum(sum(tmp_cent_map));
                
            elseif (Selection == 2) || (Selection == 4)
                % prediction step
                tmp_cent_map2 = zeros(size(tmp_cent_map));
                for k = 1:size(pt,1)
                    tmp_cent_map2 = tmp_cent_map2+upd_cell1{k,model_idx}*tmp_cent_map(pt(k,1),pt(k,2));
                end
                tmp_cent_map = tmp_cent_map2;
                
                % update step
                for i=1:NumOfRobot
                    tmp_cent_map = tmp_cent_map.*rbt_cent.prob{i};
                end
                rbt_cent.map = tmp_cent_map/sum(sum(tmp_cent_map));
            end
        end
        
        function rbt = predStep(rbt,inPara)
            %% Probability map update based on target motion model
            % calculate the prediction matrix
            % this part takes too much time, needs to optimize some time
            %%% note: in fact, this method may be too dull, can think about a
            %%% clever one that does not need so much storage space.
            
            % target motion model
            u_set = inPara.u_set;
            V_set = inPara.V_set;
                                   
            [ptx,pty] = meshgrid(1:fld.x,1:fld.y);
            pt = [ptx(:),pty(:)];
            rbt.upd_cell = cell(size(pt,1),fld.target.mode_num); % pred matrix for all motion models
            
            for mode_cnt = 1:fld.target.mode_num
                for ii = 1:size(pt,1)
                    % transition matrix
                    tmp_trans = zeros(fld.x,fld.y);
                    mu = pt(ii,:)'+u_set(:,ii);
                    for x = 1:fld.x
                        for y = 1:fld.y
                            tmp_trans(x,y) = mvncdf([x-1;y-1],[x,y],mu,V_set);                            
                        end
                    end
                    rbt.upd_cell{ii,mode_cnt} = tmp_trans;
                end          
            end
            
%             for ii = 1:size(pt,1)
%                 for jj = 1:size(fld.target.dx_set,2)
%                     %             fld.target.dx = fld.target.dx_set(jj);
%                     %             fld.target.dy = fld.target.dy_set(jj);
%                     tmp_dx = fld.target.dx_set(jj);
%                     tmp_dy = fld.target.dy_set(jj);
%                     
%                     upd_cell1{ii,jj} = zeros(fld.x,fld.y);
%                     if (pt(ii,1)+fld.target.speed*tmp_dx <= fld.x) && (pt(ii,2)+fld.target.speed*tmp_dy <= fld.y) && (pt(ii,1)+fld.target.speed*tmp_dx >= 1) && (pt(ii,2)+fld.target.speed*tmp_dy >= 1)
%                         upd_cell1{ii,jj}(pt(ii,1)+fld.target.speed*tmp_dx,pt(ii,2)+fld.target.speed*tmp_dy) = 1;
%                     end
%                 end
%             end
            
        end
        
        function rbt = stepUpdate(rbt)
           rbt.step_cnt = rbt.step_cnt+1; 
        end
    end
end