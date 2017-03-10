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
            
            tmp_u_set = this.target.u_set;
            tmp_v = this.target.V_set;
            tmp_idx = this.target.model_idx;
            if this.tar_move == 1
                % Target Moves
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
%                 this.target.pos = tmp_pos;
                this.target.pos = (mvnrnd(tmp_pos',tmp_v))';
                this.target.traj = [this.target.traj,tmp_pos];
                this.target.model_idx = tmp_idx;
                
            end
        end
    end   
end