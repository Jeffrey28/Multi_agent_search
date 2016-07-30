mode_num = 4;

u_set = [[1;1],[-1;-1],[1;-1],[-1;1]]; %inPara.u_set; 
V_set = 0.01*eye(2); % 

fld_size = [100;100];

[ptx,pty] = meshgrid(0.5:1:fld_size(1)+0.5,0.5:1:fld_size(2)+0.5);
pt = [ptx(:),pty(:)];
upd_matrix = cell(mode_num,1); % pred matrix for all motion models
for mode_cnt = 1:mode_num
    % tmp_matrix(ii,:) is the transition probability P(x^i_k+1|x^j_k) for    
    % all x^j_k in the grid    
    trans_mat = zeros(fld_size(1)*fld_size(2));
    count = 1;    
    for x = 1:fld_size(1)        
        for y = 1:fld_size(2)        
            display([x;y])
            mu = [x;y]+u_set(:,mode_cnt);
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

save('upd_matrix.mat','upd_matrix');