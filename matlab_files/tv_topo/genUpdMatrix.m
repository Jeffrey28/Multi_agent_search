% pre-generate the update matrix corresponding to the prediction step in
% filtering

%% linear target
%{
mode_num = 4;

u_set = [[1;1],[-1;-1],[1;-1],[-1;1]]; %inPara.u_set; 
V_set = 0.1*eye(2); % 

fld_size = [100;100];

[ptx,pty] = meshgrid(0.5:1:fld_size(1)+0.5,0.5:1:fld_size(2)+0.5);
% pt = [ptx(:),pty(:)];
upd_matrix = cell(mode_num,1); % pred matrix for all motion models
for mode_cnt = 1:mode_num
    display(mode_num)
    % tmp_matrix(ii,:) is the transition probability P(x^i_k+1|x^j_k) for    
    % all x^j_k in the grid    
    trans_mat = zeros(fld_size(1)*fld_size(2));
    count = 1;    
    for x = 1:fld_size(1)        
        for y = 1:fld_size(2)        
            display([x;y])
            sprintf('progress: %d',count/(fld_size(1)*fld_size(2)))
            mu = [x;y]+u_set(:,mode_cnt);
            % transition matrix            
            % tmp_trans(x,y) shows the transition probability P(x^i_k+1|[x;y]),            
            % considering the dynamic model of vehicle            
            % tmp_trans = zeros(fld_size(1),fld_size(2)); 
            % read trans_mat in column, so trans_mat(:,1) is the transition
            % matrix from (1,1) point to all other points.
            tmp_trans = mvncdf([ptx(:),pty(:)],mu',V_set); % transition along the y direction, i.e., (1,1),(1,2),...,(2,1),...
            tmp_trans_mat = (reshape(tmp_trans,fld_size(2)+1,fld_size(1)+1))'; % after the reshape, the matrix shows the transition to [(1,1) (1,2) ... (1,N); (2,1), ...]
            tmp_trans_mat2 = tmp_trans_mat(2:end,2:end)-tmp_trans_mat(1:end-1,2:end)-tmp_trans_mat(2:end,1:end-1)+tmp_trans_mat(1:end-1,1:end-1);
            tmp_trans_mat2 = tmp_trans_mat2';
            trans_mat(:,count) = tmp_trans_mat2(:); % transition probability along the x direction, i.e., [(1,1);(2,1);...;(N,1);(1,2),...]
            count = count + 1;                        
        end        
    end
    upd_matrix{mode_cnt} = trans_mat;
end

save('upd_matrix_v01.mat','upd_matrix','-v7.3');
%}

%% sinusoidal target
%{
u_set = [[-1;1],[1;1],[-1;-1],[1;-1]];
V_set = 0.1*eye(2); % 
mode_num = size(u_set,2);
dt = 1;

fld_size = [100;100];

[ptx,pty] = meshgrid(0.5:1:fld_size(1)+0.5,0.5:1:fld_size(2)+0.5);
% pt = [ptx(:),pty(:)];
upd_matrix = cell(mode_num,1); % pred matrix for all motion models
for mode_cnt = 1:mode_num
    display(mode_num)
    % tmp_matrix(ii,:) is the transition probability P(x^i_k+1|x^j_k) for    
    % all x^j_k in the grid    
    trans_mat = zeros(fld_size(1)*fld_size(2));
    count = 1;    
    for x = 1:fld_size(1)        
        for y = 1:fld_size(2)        
            display([x;y])
            sprintf('progress: %d',count/(fld_size(1)*fld_size(2)))
%             mu = [x;y]+u_set(:,mode_cnt)*dt;
            tmp_x = x + u_set(1,mode_cnt)*dt;
            tmp_y = y + u_set(2,mode_cnt)*cos(0.2*x)*dt;
            mu = [tmp_x;tmp_y];
            
            % transition matrix            
            % tmp_trans(x,y) shows the transition probability P(x^i_k+1|[x;y]),            
            % considering the dynamic model of vehicle            
            % tmp_trans = zeros(fld_size(1),fld_size(2));            
            tmp_trans = mvncdf([ptx(:),pty(:)],mu',V_set);
            tmp_trans_mat = (reshape(tmp_trans,fld_size(2)+1,fld_size(1)+1))';
            tmp_trans_mat2 = tmp_trans_mat(2:end,2:end)-tmp_trans_mat(1:end-1,2:end)-tmp_trans_mat(2:end,1:end-1)+tmp_trans_mat(1:end-1,1:end-1);
            tmp_trans_mat2 = tmp_trans_mat2';            
            trans_mat(:,count) = tmp_trans_mat2(:);
            count = count + 1;                        
        end        
    end
    upd_matrix{mode_cnt} = trans_mat;
end

save('upd_matrix_sin_v01.mat','upd_matrix','-v7.3');
%}

%% circular target
%
center_set = [[50.5;50.5],[50.5;150.5],[50.5;-150.5],[-50.5;50.5],[150.5;50.5]];
V_set = 0.1*eye(2); % 
mode_num = size(center_set,2);
dt = 1;

fld_size = [100;100];

des_lin_vel = 2; % desired linear velocity
[ptx,pty] = meshgrid(0.5:1:fld_size(1)+0.5,0.5:1:fld_size(2)+0.5);
pt = [ptx(:),pty(:)];
upd_matrix = cell(mode_num,1); % pred matrix for all motion models
for mode_cnt = 1:mode_num
    display(mode_num)
    count = 1;
    % tmp_matrix(ii,:) is the transition probability P(x^i_k+1|x^j_k) for    
    % all x^j_k in the grid    
    trans_mat = zeros(fld_size(1)*fld_size(2));
    
    cetr = center_set(:,mode_cnt);
    for x = 1:fld_size(1)        
        for y = 1:fld_size(2)        
            display([x;y])
            sprintf('progress: %d',count/(fld_size(1)*fld_size(2)))
            
            radius = norm([x;y]-cetr);
            ang_vel = des_lin_vel/radius;
            d_ang = ang_vel*dt; % the angle increment
            cur_ang = atan2(y-cetr(2),x-cetr(1));
            
            tmp_x = x - radius*sin(cur_ang)*d_ang;
            tmp_y = y + radius*cos(cur_ang)*d_ang;
            mu = [tmp_x;tmp_y];
            
            % transition matrix            
            % tmp_trans(x,y) shows the transition probability P(x^i_k+1|[x;y]),            
            % considering the dynamic model of vehicle            
            % tmp_trans = zeros(fld_size(1),fld_size(2));            
            tmp_trans = mvncdf([ptx(:),pty(:)],mu',V_set);
            tmp_trans_mat = (reshape(tmp_trans,fld_size(2)+1,fld_size(1)+1))';
            tmp_trans_mat2 = tmp_trans_mat(2:end,2:end)-tmp_trans_mat(1:end-1,2:end)-tmp_trans_mat(2:end,1:end-1)+tmp_trans_mat(1:end-1,1:end-1);
            tmp_trans_mat2 = tmp_trans_mat2';
            trans_mat(:,count) = tmp_trans_mat2(:);
            count = count + 1;
        end        
    end
    upd_matrix{mode_cnt} = trans_mat;
end

save('upd_matrix_cir_v01.mat','upd_matrix','-v7.3');
%}