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
        state; % state = [position;heading]. used in sonar sensor model
        % used for circular motion of robot
        T; % period of circular motion
        r; % radius
        w; % angular velocity
        center; % center of the circular motion
        
        % sensor specs
        sensor_type;
        % binary sensor
        sen_cov;
        inv_sen_cov;
        sen_offset;
        % range-only sensor
        cov_ran; % coveriance of uncertainty 
        dist_ran; % sensoring range
        offset_ran; 
        % bearing-only sensor
        cov_brg;
        offset_brg;
        % range-bearing sensor
        cov_ranbrg;
        dist_ranbrg;
        % P3DX sonar sensor
        dist_sonar;
        cov_sonar;
        ang_sonar;
        
        % observation
        z; % observation measurement
        k; % measurement time
        % an array saving the model index of the target at each step, this 
        % is because in my current formulation the target may change model 
        % to avoid moving out of the field
        tar_mod; 
        
        % filtering
        nbhd_record; % record some useful info about neighbors, e.g. the time steps that have been used for updating the prob map
        lkhd_map; % prob matrix for a certain observation. a way to reduce computation time, sacrificing the space complexity
        upd_matrix; % cell used for updating probability map
        talign_map; % store the prob_map for observations with same tiem index, i.e. P(x|z^1_1:k,...,z^N_1:k)
        talign_t; % record the time for the time-aligned map
        dbf_map; % probability map
        cons_map; % prob map for concensus method
        cent_map; % prob map for centralized filter
        buffer; % communication buffer. a struct array, each element corresponding to the latest info for a robot
        buffer_cent; % this buffer is not for communication. It saves all observations at current time step and is used by centralized filter
        
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
%             this.pos = inPara.pos;
%             this.traj = inPara.pos;
            this.upd_matrix = inPara.upd_matrix; %%% can precompute this term and pass the lookup table to this constructor function
            this.idx = inPara.idx;
            if inPara.r_move == 0
                this.pos = inPara.pos; % sensor position
                this.traj = this.pos;
            elseif inPara.r_move == 1
                this.T = inPara.T; % period of circling motion
                this.center = inPara.center;
                this.r = inPara.r;
                this.w = inPara.w;
                this.pos = [inPara.center(1);inPara.center(2)+this.r]; % initial position is at the top of the circle
                this.traj = this.pos;
            elseif inPara.r_move == 2
                this.state = inPara.state; % sensor position
                this.traj = this.state(1:2);
            end
            
            % sensor spec
            this.tar_mod = [];
            
            this.sensor_type = inPara.sensor_type;
%             this.sen_cov = inPara.sen_cov;
%             this.inv_sen_cov = inPara.inv_sen_cov;
%             this.sen_offset = inPara.sen_offset;
            this.cov_ran = inPara.cov_ran;
            this.dist_ran = inPara.dist_ran;
            this.offset_ran = inPara.offset_ran;
            this.cov_brg = inPara.cov_brg;
            this.offset_brg = inPara.offset_brg;
            this.cov_ranbrg = inPara.cov_ranbrg;
            this.dist_ranbrg = inPara.dist_ranbrg;
            
            % if we use exp data
            if isfield(inPara,'dist_sonar')                
                this.dist_sonar = inPara.dist_sonar;
                this.cov_sonar = inPara.cov_sonar;
                this.ang_sonar = inPara.ang_sonar;
            end
            
            this.dbf_map = ones(inPara.fld_size(1),inPara.fld_size(2));
            this.dbf_map = this.dbf_map/sum(sum(this.dbf_map));
            this.cons_map = this.dbf_map;
            this.cent_map = this.dbf_map;
            this.talign_map = this.dbf_map; % store the prob_map for observations with same tiem index, i.e. P(x|z^1_1:k,...,z^N_1:k)
            this.talign_t = 0; % record the time for the time-aligned map
            this.lkhd_map = zeros(inPara.fld_size(1),inPara.fld_size(2));            
            this.nbhd_idx = inPara.nbhd_idx;
            
            % initialize buffer
            this.buffer(inPara.num_robot).pos = [];
            this.buffer(inPara.num_robot).z = [];
            this.buffer(inPara.num_robot).k = [];
            this.buffer(inPara.num_robot).lkhd_map = {};
            this.buffer(inPara.num_robot).used = [];   
            this.buffer_cent.pos = [];
            this.buffer_cent.z = [];
            this.buffer_cent.k = [];
            this.buffer_cent.lkhd_map = {};
            this.num_robot = inPara.num_robot;
            this.step_cnt = 0;
        end
        
        %% sensor modeling
        % generate a random measurment and computes the probability
        % likelihood map
        
        %% Binary sensor
        function this = sensorGenBin(this,fld)
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
            tmp_lkhd_map = this.sensorProbBin(fld);
            if tmp_z == 0
                tmp_lkhd_map = 1-tmp_lkhd_map;
            end
            
            this.lkhd_map = tmp_lkhd_map;
        end
        
        % computes probability likelihood map
        function lkhd_map = sensorProbBin(this,fld)
            x_r = this.pos;
            inv_cov = this.inv_sen_cov;
            offset = this.sen_offset;
            
            xlen = fld.fld_size(1);
            ylen = fld.fld_size(2);
            [ptx,pty] = meshgrid(1:xlen,1:ylen);
            pt = [ptx(:)';pty(:)'];
            pt = bsxfun(@plus,pt,offset);
            pt = bsxfun(@minus,pt,x_r);
            tmp = pt'*inv_cov*pt;
            tmp_diag = diag(tmp);
            lkhd_map = exp(-1/2*(reshape(tmp_diag,ylen,xlen))');% note meshgrid first goes along y and then x direction.
        end
        
        %% Range-only sensor
        function this = sensorGenRan(this,fld)
            % generate sensor measurement
            x_r = this.pos;
            x_t = fld.target.pos;
            cov_ran = this.cov_ran;
            dist_ran = this.dist_ran;
            offset = this.offset_ran;
            new_mean = norm(x_t-x_r)+offset;
            if norm(x_t-x_r) <= dist_ran
                this.z = normrnd(new_mean,cov_ran);
            else
                this.z = -100;
            end
               
            this.k = this.step_cnt;
            
            % generate the likelihood map for all possible target locations
            this.lkhd_map = this.sensorProbRan(fld);
        end
        
        % computes probability likelihood map
        function lkhd_map = sensorProbRan(this,fld)
            x_r = this.pos;
            z = this.z;
            cov_ran = this.cov_ran;
            dist_ran = this.dist_ran;
            
            xlen = fld.fld_size(1);
            ylen = fld.fld_size(2);
            [ptx,pty] = meshgrid(1:xlen,1:ylen);
            pt = [ptx(:)';pty(:)'];            
            pt = bsxfun(@minus,pt,x_r);
            dist = sqrt(sum(pt.^2,1));
            % find the points that are within the sensor range
            tmp_idx = (dist <= dist_ran);
            in_range_dist = dist(tmp_idx);
            
            if z ~= -100
                tmp_lkhd = normpdf(in_range_dist,z,cov_ran);
                lkhd_map = zeros(1,length(ptx(:)));
                lkhd_map(tmp_idx) = tmp_lkhd;
            else
                lkhd_map = ones(1,length(ptx(:)));
                lkhd_map(tmp_idx) = 0;
            end
            lkhd_map = (reshape(lkhd_map,ylen,xlen))';            
        end
        
        %% Bearing-only sensor
        function this = sensorGenBrg(this,fld)
            % generate sensor measurement
            x_r = this.pos;
            x_t = fld.target.pos;
            tmp_vec = x_t-x_r;                      
            cov_brg = this.cov_brg;
            % consider offset
            offset = this.offset_brg;
            new_mean = atan2(tmp_vec(2),tmp_vec(1))+offset;
            this.z = normrnd(new_mean,cov_brg);              
            this.k = this.step_cnt;
            
            % generate the likelihood map for all possible target locations
            this.lkhd_map = this.sensorProbBrg(fld);
        end
        
        % computes probability likelihood map
        function lkhd_map = sensorProbBrg(this,fld)
            x_r = this.pos;
            z = this.z;
            cov_brg = this.cov_brg;            
            
            xlen = fld.fld_size(1);
            ylen = fld.fld_size(2);
            [ptx,pty] = meshgrid(1:xlen,1:ylen);
            pt = [ptx(:)';pty(:)'];            
            pt = bsxfun(@minus,pt,x_r);
            angles = atan2(pt(2,:),pt(1,:))';
            
            tmp_lkhd = normpdf(angles,z,cov_brg);
            lkhd_map = (reshape(tmp_lkhd,ylen,xlen))';
        end
        
        %% Range-Bearing sensor
        function this = sensorGenRanBrg(this,fld)
            % generate sensor measurement
            x_r = this.pos;
            x_t = fld.target.pos;
            tmp_vec = x_t-x_r;                      
            cov_ranbrg = this.cov_ranbrg;
            dist_ranbrg = this.dist_ranbrg;
            
            if norm(x_t-x_r) <= dist_ranbrg
                this.z = (mvnrnd([tmp_vec(1),tmp_vec(2)],cov_ranbrg))';
            else
                this.z = [-100;-100];
            end
            
            this.k = this.step_cnt;
            
            % generate the likelihood map for all possible target locations
            this.lkhd_map = this.sensorProbRanBrg(fld);
        end
        
        % computes probability likelihood map
        function lkhd_map = sensorProbRanBrg(this,fld)
            x_r = this.pos;
            z = this.z;
            cov_ranbrg = this.cov_ranbrg;
            dist_ranbrg = this.dist_ranbrg;
            
            xlen = fld.fld_size(1);
            ylen = fld.fld_size(2);
            [ptx,pty] = meshgrid(1:xlen,1:ylen);
            pt = [ptx(:)';pty(:)'];            
            pt = bsxfun(@minus,pt,x_r);
            dist = sqrt(sum(pt.^2,1));
            % find the points that are within the sensor range
            tmp_idx = (dist <= dist_ranbrg);
%             in_range_dist = dist(tmp_idx);
            
            if (z(1) ~= -100) && (z(2)~= -100)
                tmp_lkhd = mvnpdf(pt(:,tmp_idx)',z',cov_ranbrg);
                lkhd_map = zeros(1,length(ptx(:)));
                lkhd_map(tmp_idx) = tmp_lkhd;
            else
                lkhd_map = ones(1,length(ptx(:)));
                lkhd_map(tmp_idx) = 0;
            end
            lkhd_map = (reshape(lkhd_map,ylen,xlen))';            
        end
        
        %% P3DX onboard sonar
        function this = sensorGenSonar(this,fld,meas)
            this.z = meas;
            this.k = this.step_cnt;            
            % generate the likelihood map for all possible target locations
            this.lkhd_map = this.sensorProbSonar(fld);
        end
        
        function lkhd_map = sensorProbSonar(this,fld)
            x_r = this.state;
            z = this.z;
            cov_sonar = this.cov_sonar;
            dist_sonar = this.dist_sonar;
            ang_sonar = this.ang_sonar;
            
            xlen = fld.fld_size(1);
            ylen = fld.fld_size(2);
            [ptx,pty] = meshgrid(1:xlen,1:ylen);
            pt = [ptx(:)';pty(:)'];            
            pt = bsxfun(@minus,pt,x_r(1:2));
            dist = sqrt(sum(pt.^2,1));
            ang = atan2(pt(2,:),pt(1,:));
%             tmp_idx = ang<0; % if -pi <= ang <= 0, make it within [pi,2*pi]
%             ang(tmp_idx) = ang(tmp_idx)+2*pi;            
            ang = ang-x_r(3); % relative heading angle from the center to the target           
            
            % find the points that are within the sensor range and FOV
            tmp_idx = (dist <= dist_sonar) & (sin(ang)>=-sin(ang_sonar/2))...
                & (sin(ang)<=sin(ang_sonar/2))& (cos(ang)>=cos(ang_sonar/2));%(ang>=-ang_sonar/2) & (ang<=ang_sonar/2);
            in_range_dist = dist(tmp_idx);
            
            if (z ~= -100)
                tmp_lkhd = normpdf(in_range_dist,z,cov_sonar);
                lkhd_map = 0.01*ones(1,length(ptx(:)));
                lkhd_map(tmp_idx) = tmp_lkhd;
            else
                lkhd_map = ones(1,length(ptx(:)));
                lkhd_map(tmp_idx) = 0;
            end
            lkhd_map = (reshape(lkhd_map,ylen,xlen))';            
        end
        
        
        %% data storage and update 
        function this = updOwnMsmt(this,inPara)
            % (1) self-observation
            % update robot's own measurement in the communication buffer
            
            selection = inPara.selection;
            if (selection == 1) || (selection == 3)
                % static target
                % update the buffer with the robot's own observation
                this.buffer(this.idx).pos = this.pos;
                this.buffer(this.idx).z = this.z;
                this.buffer(this.idx).k = this.step_cnt;
                this.buffer(this.idx).lkhd_map = this.lkhd_map;
                
            elseif (selection == 2) || (selection == 4)
                % moving target
                this.buffer(this.idx).pos = [this.buffer(this.idx).pos,this.pos];
                this.buffer(this.idx).z = [this.buffer(this.idx).z,this.z];
                this.buffer(this.idx).k = [this.buffer(this.idx).k,this.k];
                this.buffer(this.idx).lkhd_map{this.step_cnt} = this.lkhd_map;             
            end
        end
        
        function this = dataExch(this,inPara)
            % (2) data transmission
            % exchange communication buffer with neighbors
            selection = inPara.selection;
            rbt_nbhd_set = inPara.rbt_nbhd_set;
            if (selection == 1) || (selection == 3)
                % static target
                % exchange and update buffer
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
                end                
                
            elseif (selection == 2) || (selection == 4)
                % moving target
                for t = 1:length(rbt_nbhd_set)
                    % note: communication only transmit the latest
                    % observation stored in each neighbor
                    tmp_rbt = rbt_nbhd_set{t};
                    for jj = 1:this.num_robot
                        if (~isempty(tmp_rbt.buffer(jj).k)) && (isempty(this.buffer(jj).k) || (this.buffer(jj).k(end) < tmp_rbt.buffer(jj).k(end)))
                            %%% this code only handles the fixed topology
                            %%% case, i.e. each time a one-step newer
                            %%% observation is received. for multi-step
                            %%% newer observations, such code needs
                            %%% modification.
                            this.buffer(jj).pos = [this.buffer(jj).pos,tmp_rbt.buffer(jj).pos(:,end)];
                            this.buffer(jj).z = [this.buffer(jj).z,tmp_rbt.buffer(jj).z(end)];
                            this.buffer(jj).k = [this.buffer(jj).k,tmp_rbt.buffer(jj).k(end)];
                            % in real experiment, this prob term should not
                            % be communicated. In simulation, this is for
                            % the purpose of accelerating the computation speed.
                            if isempty(this.buffer(jj).lkhd_map)                                
                                this.buffer(jj).lkhd_map = tmp_rbt.buffer(jj).lkhd_map(end);
                            else
                                this.buffer(jj).lkhd_map(end+1) = tmp_rbt.buffer(jj).lkhd_map(end);
                            end
                        end
                    end
                end
            end
        end
        
        %% filters
        function this = DBF(this,inPara)
            % filtering
            selection = inPara.selection;
%             target_model = inPara.target_model;
            if (selection == 1) || (selection == 3)
                % calculate probility of latest z               
                for jj=1:this.num_robot % Robot Iteration
                    if (~isempty(this.buffer(jj).k)) && (~ismember(this.buffer(jj).k,this.buffer(jj).used))
                        this.dbf_map=this.dbf_map.*this.buffer(jj).lkhd_map;
                        this.buffer(jj).used = [this.buffer(jj).used,this.buffer(jj).k];
                    end
                end
                this.dbf_map=this.dbf_map/sum(sum(this.dbf_map));
                                
            elseif (selection == 2) || (selection == 4)
%                 upd_matrix = this.upd_matrix{target_model};
                %% update by bayes rule
                % note: main computation resource are used in calling sensorProbBin function.
                % when using grid map, can consider precomputing
                % results and save as a lookup table
               
                talign_flag = 1; % if all agent's observation's time are no less than talign_t+1, then talign_flag = 1, increase talign_t
                tmp_t = this.talign_t;
                tmp_map = this.talign_map; % time-aligned map
                
%                 display(this.idx)
                for t = (this.talign_t+1):this.step_cnt
                    display(t)
                    upd_matrix = this.upd_matrix{this.tar_mod(t)};
                    %% one-step prediction step                     
                    % p(x_k+1) = sum_{x_k} p(x_k+1|x_k)p(x_k)
                    % note, data in upd_matrix is first along y direction 
                    % and then along x direction. therefore, when using
                    % tmp_map(:), care should be taken since tmp_map(:)
                    % orders data first along x direction, then y
                    % direction.
                    tmp_map = tmp_map';
                    tmp_map2 = upd_matrix*tmp_map(:);
                    tmp_map = (reshape(tmp_map2,size(tmp_map)))';
                    
                    %% updating step                                       
                    for jj=1:this.num_robot
                        display(jj)
                        if (~isempty(this.buffer(jj).k)) && (this.buffer(jj).k(end) >= t)
                            % note: this update is not valid in real
                            % experiment since we don't communicate
                            % probability. This line of code is for
                            % computation reduction in simulation
                            tmp_map = tmp_map.*this.buffer(jj).lkhd_map{t};
                        else
                            talign_flag = 0;
                        end
                    end
                                        
                    % after the first loop, the robot's aligned time
                    % increase by one if the robot's buffer is no later
                    % than the previous aligned time
                    if (t == this.talign_t+1) && (talign_flag == 1)
                        this.talign_map = tmp_map;
                        this.talign_map = this.talign_map/sum(sum(this.talign_map));
                        tmp_t = tmp_t+1;
                        % when we increase the aligned time, we can remove 
                        % the previous data 
                        if this.talign_t > 0 
                            for jj=1:this.num_robot
                                this.buffer(jj).pos(:,this.talign_t) = -ones(2,1);
                                this.buffer(jj).z(this.talign_t) = -1;
                                this.buffer(jj).k(this.talign_t) = -1;
                                % note: lkhd_map is a cell, therefore we
                                % keep all the old cell (empty cell) but
                                % the length of lkhd_map will not decrease.
                                % For pos, z, k, they are arrays, so their
                                % length is constant (except the first few
                                % steps). This should be noticed when
                                % deciding the index of the element to
                                % be removed
                                this.buffer(jj).lkhd_map{this.talign_t} = [];
                            end
                        end
                    end
                end
                
                this.talign_t = tmp_t;
                this.dbf_map = tmp_map;
                this.dbf_map = this.dbf_map/sum(sum(this.dbf_map));
            end
        end
        
        function this = updMap(this,inPara)
            % update own map using its own measurement, used for consensus filter
            selection = inPara.selection;
            target_model = inPara.target_model;
            
            % consensus map
            if (selection == 1) || (selection == 3)
                this.cons_map = this.cons_map.*this.lkhd_map;
            elseif (selection == 2) || (selection == 4)
                upd_matrix = this.upd_matrix{target_model};
                tmp_map = (this.cons_map)';
                tmp_map2 = upd_matrix*tmp_map(:);
                tmp_map = (reshape(tmp_map2,size(tmp_map)))';
                this.cons_map = tmp_map.*this.lkhd_map;
            end
            this.cons_map = this.cons_map/sum(sum(this.cons_map));
        end
        
        function this = cons(this,inPara)
            %% %%%%%%%%%%%%%%  Consensus Method %%%%%%%%%%%%%%%%%%
            % steps:
            % (1) observe and update the probability map for time k
            % (2) send/receive the probability map for time k from neighbors
            % (3) repeat step (1)            
            
            rbt_nbhd_set = inPara.rbt_nbhd_set;
            cons_fig = inPara.cons_fig;
            % consensus step
            % receive and weighted average neighboring maps            
             
            neigh_num=length(rbt_nbhd_set);            
            for t = 1:neigh_num
                this.cons_map = this.cons_map +rbt_nbhd_set{t}.cons_map;
            end
            this.cons_map = this.cons_map/(neigh_num+1);

            % plot local PDFs after concensus
            
            if cons_fig
                figure
%                 subplot(2,3,i); 
                contourf((this.cons_map)'); title(['Sensor ',this.idx]);
                % hold on;                
                plot(this.pos(1), this.pos(2), 's','MarkerSize',8,'LineWidth',3);       
            end
        end
        
        function this = CF(this,inPara)
            %% %%%%%%%%%%%%%% Centralized BF %%%%%%%%%%%%%%%%%%
            % steps:
            % (1) receive all robots' observations
            % (2) update the probability map for time k
            % (3) repeat step (1)
            
            selection = inPara.selection;
            target_model = inPara.target_model;
            
            if (selection == 1) || (selection == 3)
                for ii = 1:this.num_robot
                    this.cent_map = this.cent_map.*this.buffer_cent.lkhd_map{ii};
                end
            elseif (selection == 2) || (selection == 4)
                % prediction step
                upd_matrix = this.upd_matrix{target_model};
                tmp_map = this.cent_map;
                tmp_map2 = upd_matrix*tmp_map(:);
                tmp_map = reshape(tmp_map2,size(tmp_map));
                
                % update step
                for ii = 1:this.num_robot
                    tmp_map = tmp_map.*this.buffer_cent.lkhd_map{ii};
                end
                this.cent_map = tmp_map;
            end
            this.cent_map = this.cent_map/sum(sum(this.cent_map)); 
        end      
                
        %% robot motion
        function this = robotMove(this)
            tmp_angl = atan2(this.pos(2)-this.center(2),this.pos(1)-this.center(1));
            tmp_angl = tmp_angl+this.w;
            this.pos = this.r*[cos(tmp_angl);sin(tmp_angl)]+this.center;
            this.traj = [this.traj,this.pos];
        end
        
        %% metrics
        function this = computeMetrics(this,fld,id)
            % Computing Performance Metrics
            count = this.step_cnt;
            
            %% ML error
            % DBF
            if strcmp(id,'dbf')
                [tmp_x1,tmp_y1] = find(this.dbf_map == max(this.dbf_map(:)));
                if length(tmp_x1) > 1
                    tmp_idx = randi(length(tmp_x1),1,1);
                else
                    tmp_idx = 1;
                end
                this.ml_pos_dbf(:,count) = [tmp_x1(tmp_idx);tmp_y1(tmp_idx)];
                this.ml_err_dbf(count) = norm(this.ml_pos_dbf(:,count)-fld.target.pos);
            end
            
            % concensus
            if strcmp(id,'cons')
                [tmp_x2,tmp_y2] = find(this.cons_map == max(this.cons_map(:)));
                if length(tmp_x2) > 1
                    tmp_idx2 = randi(length(tmp_x2),1,1);
                else
                    tmp_idx2 = 1;
                end
                this.ml_pos_cons(:,count) = [tmp_x2(tmp_idx2);tmp_y2(tmp_idx2)];
                this.ml_err_cons(count) = norm(this.ml_pos_cons(:,count)-fld.target.pos);
            end
            
            % centralized
            if strcmp(id,'cent')
                [tmp_x3,tmp_y3] = find(this.cent_map == max(this.cent_map(:)));
                if length(tmp_x3) > 1
                    tmp_idx3 = randi(length(tmp_x3),1,1);
                else
                    tmp_idx3 = 1;
                end
                this.ml_pos_cent(:,count) = [tmp_x3(tmp_idx3);tmp_y3(tmp_idx3)];
                this.ml_err_cent(count) = norm(this.ml_pos_cent(:,count)-fld.target.pos);
            end
            
            %% Covariance of posterior pdf
            [ptx,pty] = meshgrid(1:fld.fld_size(1),1:fld.fld_size(2));
            pt = [ptx(:),pty(:)];
            
            % DBF
            if strcmp(id,'dbf')
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
            end
            
            if strcmp(id,'cons')
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
            if strcmp(id,'cent')
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
            end
            
            %% Entropy of posterior pdf
            % DBF
            if strcmp(id,'dbf')
                tmp_map1 = this.dbf_map;
                % this avoids the error when some grid has zeros probability
                tmp_map1(tmp_map1 <= realmin) = realmin;
                dis_entropy = -(tmp_map1).*log2(tmp_map1); % get the p*log(p) for all grid points
                this.ent_dbf(count) = sum(sum(dis_entropy));
            end
            
            % concensus
            if strcmp(id,'cons')
                tmp_map2 = this.cons_map;
                % this avoids the error when some grid has zeros probability
                tmp_map2(tmp_map2 <= realmin) = realmin;
                dis_entropy = -(tmp_map2).*log2(tmp_map2); % get the p*log(p) for all grid points
                this.ent_cons(count) = sum(sum(dis_entropy));
            end
            
            % centralized
            if strcmp(id,'cent')
                tmp_map3 = this.cent_map;
                % this avoids the error when some grid has zeros probability
                tmp_map3(tmp_map3 <= realmin) = realmin;
                dis_entropy = -(tmp_map3).*log2(tmp_map3); % get the p*log(p) for all grid points
                this.ent_cent(count) = sum(sum(dis_entropy));
            end
        end        
    end
end