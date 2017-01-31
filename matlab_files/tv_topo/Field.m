classdef Field
    properties
        map; % field map
        fld_size; % length of x,y coordinate  
        target; % a struct for target   
        tar_move; % indicator whether the target moves
        dt; % simulation time interval
    end
    
    methods
        function obj = Field(inPara)
            obj.fld_size = inPara.fld_size;
            obj.map = ones(obj.fld_size(1),obj.fld_size(2))/(obj.fld_size(1)*obj.fld_size(2));
            obj.target = inPara.target;
            obj.tar_move = inPara.tar_move;
            obj.dt = inPara.dt;
        end
            
        function this = targetMove(this)                        
            tmp_idx = this.target.model_idx;
            if this.tar_move == 1
                % Target Moves
                
                % linear target model
                % x_k+1 = x_k+vx
                % y_k+1 = y_k+vy
                if strcmp(this.target.mode,'linear')
                    tmp_u_set = this.target.u_set;
                    %                 tmp_v = this.target.V_set;
                    % if current model makes target out of field, choose the next
                    % motion model in fld.target.dx_set and fld.target.dy_set
                    tmp_pos = this.target.pos + tmp_u_set(:,tmp_idx)*this.dt;
                    %                 tmp_pos = this.target.pos + tmp_u_set(:,tmp_idx)*this.dt+(mvnrnd([0,0],tmp_v))';
                    while (tmp_pos(1) <= 0 || tmp_pos(2) <= 0 || tmp_pos(1) >= this.fld_size(1) || tmp_pos(2) >= this.fld_size(2))
                        tmp_idx = rem(tmp_idx+1,length(this.target.u_set));
                        if tmp_idx == 0
                            tmp_idx = length(this.target.u_set);
                        end
                        tmp_pos = this.target.pos + tmp_u_set(:,tmp_idx)*this.dt;
                    end
                    this.target.pos = tmp_pos;
                    this.target.traj = [this.target.traj,tmp_pos];
                    this.target.model_idx = tmp_idx;
                
                % sinusoidal model
                % x_k+1 = x_k+vx
                % y_k+1 = y_k+vy*cos(w*x_k)
                % this model is derived from : y=vy/vx*sin(x). Let
                % \dot{x}=v_x, then \dot(y)=vy*cos(x). note, vx and vy are
                % constants.
                elseif strcmp(this.target.mode,'sin')
                    tmp_u_set = this.target.u_set;
                    %                 tmp_v = this.target.V_set;
                    tmp_x = this.target.pos(1) + tmp_u_set(1,tmp_idx)*this.dt;
                    tmp_y = this.target.pos(2) + tmp_u_set(2,tmp_idx)*cos(0.2*this.target.pos(1))*this.dt;
                    tmp_pos = [tmp_x;tmp_y];
                    
                    while (tmp_pos(1) <= 0 || tmp_pos(2) <= 0 || tmp_pos(1) >= this.fld_size(1) || tmp_pos(2) >= this.fld_size(2))
                        tmp_idx = rem(tmp_idx+1,length(this.target.u_set));
                        if tmp_idx == 0
                            tmp_idx = length(this.target.u_set);
                        end
                        tmp_x = this.target.pos(1) + tmp_u_set(1,tmp_idx)*this.dt;
                        tmp_y = this.target.pos(2) + tmp_u_set(2,tmp_idx)*cos(0.2*this.target.pos(1))*this.dt;
                        tmp_pos = [tmp_x;tmp_y];
                    end
                    
                    this.target.pos = tmp_pos;
                    this.target.traj = [this.target.traj,tmp_pos];
                    this.target.model_idx = tmp_idx;                
                
                elseif strcmp(this.target.mode,'circle')
                    x = this.target.pos(1);
                    y = this.target.pos(2);
                    des_lin_vel = 2;
                    
                    cetr = this.target.center_set(:,tmp_idx);
                    radius = norm([x;y]-cetr);
                    ang_vel = des_lin_vel/radius;
                    d_ang = ang_vel*this.dt; % the angle increment
                    cur_ang = atan2(y-cetr(2),x-cetr(1));
                    
                    tmp_x = x - radius*sin(cur_ang)*d_ang;
                    tmp_y = y + radius*cos(cur_ang)*d_ang;
                    tmp_pos = [tmp_x;tmp_y];
                    
                    while (tmp_pos(1) <= 0 || tmp_pos(2) <= 0 || tmp_pos(1) >= this.fld_size(1) || tmp_pos(2) >= this.fld_size(2))
                        tmp_idx = rem(tmp_idx+1,length(this.target.center_set));
                        if tmp_idx == 0
                            tmp_idx = length(this.target.center_set);
                        end
                        
                        cetr = this.target.center_set(:,tmp_idx);
                        radius = norm([x;y]-cetr);
                        ang_vel = des_lin_vel/radius;
                        d_ang = ang_vel*this.dt; % the angle increment
                        cur_ang = atan2(y-cetr(2),x-cetr(1));
                        
                        tmp_x = x - radius*sin(cur_ang)*d_ang;
                        tmp_y = y + radius*cos(cur_ang)*d_ang;
                        tmp_pos = [tmp_x;tmp_y];
                    end
                    
                    this.target.pos = tmp_pos;
                    this.target.traj = [this.target.traj,tmp_pos];
                    this.target.model_idx = tmp_idx;
                end
            end
        end
    end   
end