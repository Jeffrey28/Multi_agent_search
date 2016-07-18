classdef Robot
    properties
        % basic info
        idx; % index of the robot
        step_cnt; % current time step
        nbhd_idx; % index of neighboring robots
        num_robot;
        
        % motion state
        traj;
        pos; % robot position
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
        
        % filtering
        nbhd_record; % record some useful info about neighbors, e.g. the time steps that have been used for updating the prob map
        lkhd_map; % prob matrix for a certain observation. a way to reduce computation time, sacrificing the space complexity
        upd_cell; % cell used for updating probability map
        talign_map; % store the prob_map for observations with same tiem index, i.e. P(x|z^1_1:k,...,z^N_1:k)
        talign_t; % record the time for the time-aligned map
        dbf_map; % probability map
        cons_map; % prob map for concensus method
        cent_map; % prob map for centralized filter
        buffer; % communication buffer. a struct array, each element corresponding to the latest info for a robot
        
        % plotting
        color; % color for drawing plots
        
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
        function this = Robot(inPara)
            this.pos = inPara.pos;
            this.upd_cell = {}; %%% can precompute this term and pass the lookup table to this constructor function
            this.idx = inPara.idx;
            if inPara.r_move == 0
                this.pos = inPara.pos; % sensor position
            elseif inPara.r_move == 1
                this.T = 20; % period of circling motion
                this.center = inPara.center;
                this.r = inPara.r; %15;
                this.w = 2*pi/this.T;
                this.pos = [inPara.center(1);inPara.center(1)+this.r]; % initial position is at the top of the circle
            end
            this.traj = [];
            this.sen_cov = inPara.sen_cov;
            this.inv_sen_cov = inPara.inv_sen_cov;
            this.sen_offset = inPara.sen_offset;
            this.dbf_map = ones(inPara.fld_size(1),inPara.fld_size(2));
            this.dbf_map = this.dbf_map/sum(sum(this.dbf_map));
            this.cons_map = this.dbf_map;
            this.cen_map = this.dbf_map;
            this.talign_map = this.dbf_map; % store the prob_map for observations with same tiem index, i.e. P(x|z^1_1:k,...,z^N_1:k)
            this.talign_t = 0; % record the time for the time-aligned map
            this.lkhd_map = zeros(inPara.fld_size(1),inPara.fld_size(2));
            %             obj.entropy = zeros(1,inPara.max_step);
            this.nbhd_idx = inPara.nbhd_idx;
            % initialize buffer
            this.buffer(inPara.num_robot).pos = [];
            this.buffer(inPara.num_robot).z = [];
            this.buffer(inPara.num_robot).k = [];
            this.buffer(inPara.num_robot).lkhd_map = {};
            this.buffer(inPara.num_robot).used = [];
            %             obj.buffer = struct;
            %             if ~isempty(inPara.nbhd_idx)
            %                 obj.buffer(inPara.num_robot).used = []; % record the observations that are already used by BF
            %             end
            this.num_robot = inPara.num_robot;
            this.step_cnt = 0;
        end
        
        % generate a random measurment and computes the probability
        % likelihood map
        function this = sensorGen(this,fld)
            % generate sensor measurement
            x_r = this.pos;
            x_t = fld.target.pos;
            inv_cov = this.inv_sen_cov;
            offset = this.sen_offset;
            
            tmp_lkhd = exp(-1/2*(x_t+offset-x_r)'*inv_cov*(x_t+offset-x_r));
            tmp_z = (rand(1,1) < tmp_lkhd);
            
            this.z = tmp_z;
            this.k = this.step_cnt;
            
            % generate the likelihood map for all possible target locations
            tmp_lkhd_map = this.sensorProb(fld);
            if tmp_z == 0
                tmp_lkhd_map = 1-tmp_lkhd_map;
            end
            
            this.lkhd_map = tmp_lkhd_map;
        end
        
        % computes probability likelihood map
        function lkhd_map = sensorProb(this,fld)
            x_r = this.pos;
            inv_cov = this.inv_sen_cov;
            offset = this.sen_offset;
            
            xlen = fld.fld_size(1);
            ylen = fld.fld_size(2);
            [ptx,pty] = meshgrid(1:xlen,1:ylen);
            pt = [ptx(:)';pty(:)'];
            pt = bsxfun(@minus,pt,[0.5;0.5]); % the target position is treated as the center of each cell
            % pt = x_t+offset-x_r
            pt = bsxfun(@plus,pt,offset);
            pt = bsxfun(@minus,pt,x_r);
            %             pt = pt - x_r*ones(1,xlen*ylen);
            tmp = pt'*inv_cov*pt;
            tmp_diag = diag(tmp);
            % prob = exp(-1/2*(t-x_r)'/sigma*(t-x_r));
            lkhd_map = exp(-1/2*(reshape(tmp_diag,xlen,ylen))');
        end
        
        function this = updOwnMsmt(this,inPara)
            % update robot's own measurement in the communication buffer
            
            selection = inPara.selection;
            if (selection == 1) || (selection == 3)
                % static target
                % (1) self-observation
                % update the buffer with the robot's own observation
                this.buffer(this.idx).pos = this.pos;
                this.buffer(this.idx).z = this.z;
                this.buffer(this.idx).k = this.step_cnt;
                this.buffer(this.idx).lkhd_map = this.lkhd_map;
                
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
                this.buffer(this.idx).pos = [this.pos,this.buffer(this.idx).pos];
                this.buffer(this.idx).z = [this.z,this.buffer(this.idx).z];
                this.buffer(this.idx).k = [this.k,this.buffer(this.idx).k];
                this.buffer(this.idx).lkhd_map{this.step_cnt} = this.lkhd_map;
                
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
        
        function this = dataExch(this,inPara)
            % exchange communication buffer with neighbors
            selection = inPara.selection;
            rbt_nbhd_set = inPara.rbt_nbhd_set;
            if (selection == 1) || (selection == 3)
                % (2) exchange and update buffer
                for t = 1:length(rbt_nbhd_set)
                    % note: communication only transmit the latest
                    % observation stored in each neighbor
                    tmp_rbt = rbt_nbhd_set{t};
                    for jj = 1:this.num_robot
                        if (~isempty(tmp_rbt.buffer(jj).k)) && (isempty(this.buffer(jj).k) || (this.buffer(jj).k < tmp_rbt.buffer(jj).k))
                            this.buffer(jj).pos = tmp_rbt.buffer(jj).pos;
                            this.buffer(jj).z = tmp_rbt.buffer(jj).z;
                            this.buffer(jj).k = tmp_rbt.buffer(jj).k;
                            this.buffer(jj).lkhd_map = tmp_rbt.buffer(jj).lkhd_map;
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
                    for jj = 1:this.num_robot
                        if (~isempty(tmp_rbt.buffer(jj).k)) && (isempty(this.buffer(jj).k) || (this.buffer(jj).k < tmp_rbt.buffer(jj).k))
                            %%% this code only handles the fixed topology
                            %%% case, i.e. each time a one-step newer
                            %%% observation is received. for multi-step
                            %%% newer observations, such code needs
                            %%% modification.
                            this.buffer(jj).pos = [tmp_rbt.buffer(jj).pos(:,1),this.buffer(jj).pos];
                            this.buffer(jj).z = [tmp_rbt.buffer(jj).z(1),this.buffer(jj).z];
                            this.buffer(jj).k = [tmp_rbt.buffer(jj).k(1),this.buffer(jj).k];
                            % in real experiment, this prob term should not
                            % be communicated. In simulation, this is for
                            % the purpose of accelerating the computation speed.
                            this.buffer(jj).lkhd_map = {tmp_rbt.buffer(jj).lkhd_map{1},this.buffer(jj).lkhd_map};
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
        
        function this = DBF(this,inPara)
            selection = inPara.selection;
            if (selection == 1) || (selection == 3)
                %% update by bayes rule
                % calculate probility of latest z
                for jj=1:this.num_robot % Robot Iteration
                    if (~isempty(this.buffer(jj).k)) && (~ismember(this.buffer(jj).k,this.buffer(jj).used))
                        this.dbf_map=this.dbf_map.*this.buffer(jj).lkhd_map;
                        this.buffer(jj).used = [this.buffer(jj).used,this.buffer(jj).k];
                    end
                end
                this.dbf_map=this.dbf_map/sum(sum(this.dbf_map));
                
            elseif (selection == 2) || (selection == 4)
                upd_matrix = this.upd_matrix;
                %% update by bayes rule
                % note: main computation resource are used in calling sensorProb function.
                % when using grid map, can consider precomputing
                % results and save as a lookup table
               
                talign_flag = 1; % if all agent's observation's time are no less than talign_t+1, then talign_flag = 1, increase talign_t
                tmp_t = this.talign_t;
                tmp_map = this.talign_map; % time-aligned map
                
                for t = (this.talign_t+1):this.step_cnt
                   
                    %% one-step prediction step
%                     tmp_map2 = zeros(size(tmp_map));
%                     for k = 1:size(pt,1)
%                         tmp_map2 = tmp_map2+upd_cell{k,model_idx}*tmp_map(pt(k,1),pt(k,2));
%                     end

                    % p(x_k+1) = sum_{x_k} p(x_k+1|x_k)p(x_k)
                    tmp_map2 = upd_matrix{model_idx}*tmp_map(:);
                    tmp_map = reshape(tmp_map2,size(tmp_map));
                    
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
                    
                    for jj=1:this.num_robot
                        if (~isempty(this.buffer(jj).k)) && (this.buffer(jj).k(1) >= t)
                            % note: this update is not valid in real
                            % experiment since we don't communicate
                            % probability. This line of code is for
                            % computation reduction in simulation
                            tmp_map = tmp_map.*this.buffer(jj).lkhd_map{t};
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
                    if (t == this.talign_t+1) && (talign_flag == 1)
                        this.talign_map = tmp_map;
                        this.talign_map = this.talign_map/sum(sum(this.talign_map));
                        tmp_t = tmp_t+1;
                        % when we increase the aligned time, we can remove 
                        % the old lkhd_map 
                        for jj=1:this.num_robot
                            this.buffer(jj).lkhd_map{this.talign_t} = [];
                        end
                    end
                end
                
                this.talign_t = tmp_t;
                this.dbf_map = tmp_map;
                this.dbf_map = this.dbf_map/sum(sum(this.dbf_map));
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
                
        function rbt = stepUpdate(rbt)
            rbt.step_cnt = rbt.step_cnt+1;
        end
        
        function this = computeMetrics(this,fld,count)
            % Computing Performance Metrics
            
            % ML error
            % DBF
            [tmp_x1,tmp_y1] = find(this.dbf_map == max(this.dbf_map(:)));
            if length(tmp_x1) > 1
                tmp_idx = randi(length(tmp_x1),1,1);
            else
                tmp_idx = 1;
            end
            this.ml_pos_dbf(:,count) = [tmp_x1(tmp_idx);tmp_y1(tmp_idx)];
            this.ml_err_dbf(count) = norm(this.ml_pos_dbf(:,count)-[fld.tx;fld.ty]);
            
            % concensus
            [tmp_x2,tmp_y2] = find(this.cons_map == max(this.cons_map));
            if length(tmp_x2) > 1
                tmp_idx2 = randi(length(tmp_x2),1,1);
            else
                tmp_idx2 = 1;
            end
            this.ml_pos_cons(:,count) = [tmp_x2(tmp_idx2);tmp_y2(tmp_idx2)];
            this.ml_err_cons(count) = norm(this.ml_pos_cons(:,count)-[fld.tx;fld.ty]);
            
            % centralized
            [tmp_x3,tmp_y3] = find(this.cent_map == max(this.cent_map(:)));
            if length(tmp_x3) > 1
                tmp_idx3 = randi(length(tmp_x3),1,1);
            else
                tmp_idx3 = 1;
            end
            this.ml_pos_cent(:,count) = [tmp_x3(tmp_idx3);tmp_y3(tmp_idx3)];
            this.ml_err_cent(count) = norm(this.ml_pos_cent(:,count)-[fld.tx;fld.ty]);
            
            % Covariance of posterior pdf
            for i=1:this.num_robot
                % DBF
                tmp_map1 = this.dbf_map;
                % this avoids the error when some grid has zeros probability
                tmp_map1(tmp_map1 <= realmin) = realmin;
                
                % compute covariance of distribution
                dif1 = pt' - [(1+fld.target.pos(1))/2;(1+fld.target.pos(2))/2]*ones(1,size(pt',2));
                cov_p1 = zeros(2,2);
                for jj = 1:size(pt',2)
                    cov_p1 = cov_p1 + dif1(:,jj)*dif1(:,jj)'*tmp_map1(pt(jj,1),pt(jj,2));
                end
                this.pdf_cov_dbf{count} = cov_p1;
                this.pdf_norm_dbf(count) = norm(cov_p1,'fro');
                
                % concensus
                tmp_map2 = this.cons_map;
                % this avoids the error when some grid has zeros probability
                tmp_map2(tmp_map2 <= realmin) = realmin;
                
                % compute covariance of distribution
                dif2 = pt' - [(1+fld.target.pos(1))/2;(1+fld.target.pos(2))/2]*ones(1,size(pt',2));
                cov_p2 = zeros(2,2);
                for jj = 1:size(pt',2)
                    cov_p2 = cov_p2 + dif2(:,jj)*dif2(:,jj)'*tmp_map2(pt(jj,1),pt(jj,2));
                end
                this.pdf_cov_cons{count} = cov_p2;
                this.pdf_norm_cons(count) = norm(cov_p2,'fro');
            end
            
            % centralized
            tmp_map3 = this.cent_map;
            % this avoids the error when some grid has zeros probability
            tmp_map3(tmp_map3 <= realmin) = realmin;
            
            % compute covariance of distribution
            dif3 = pt' - [(1+fld.target.pos(1))/2;(1+fld.target.pos(2))/2]*ones(1,size(pt',2));
            cov_p3 = zeros(2,2);
            for jj = 1:size(pt',2)
                cov_p3 = cov_p3 + dif3(:,jj)*dif3(:,jj)'*tmp_map3(pt(jj,1),pt(jj,2));
            end
            this.pdf_cov_cent{count} = cov_p3;
            this.pdf_norm_cent(count) = norm(cov_p3,'fro');
            
            % Entropy of posterior pdf
            %
            for i=1:this.num_robot
                % DBF
                tmp_map1 = this.dbf_map;
                % this avoids the error when some grid has zeros probability
                tmp_map1(tmp_map1 <= realmin) = realmin;
                dis_entropy = -(tmp_map1).*log2(tmp_map1); % get the p*log(p) for all grid points
                this.ent_dbf(count) = sum(sum(dis_entropy));
                
                % concensus
                tmp_map2 = this.cons_map;
                % this avoids the error when some grid has zeros probability
                tmp_map2(tmp_map2 <= realmin) = realmin;
                dis_entropy = -(tmp_map2).*log2(tmp_map2); % get the p*log(p) for all grid points
                this.ent_cons(count) = sum(sum(dis_entropy));
            end
            % centralized
            tmp_map3 = this{1}.cent_map;
            % this avoids the error when some grid has zeros probability
            tmp_map3(tmp_map3 <= realmin) = realmin;
            dis_entropy = -(tmp_map3).*log2(tmp_map3); % get the p*log(p) for all grid points
            this.ent_cent(count) = sum(sum(dis_entropy));            
        end
        
    end
end