%% convert sim class to usually struct
% sim_for_save2 = [];
% fldnames = fieldnames(sim.sim_res);
% for ii = 1:length(fldnames)
%     sim_for_save2.(fldnames{ii}) = sim.sim_res.(fldnames{ii});
% end
% save('test_mat','sim_for_save2')

%% write data to cvs, which will be then copied for matplotlib plotting
%{
filepath = './figures/JDSMC/metrics_plot/';
filename = [filepath,'metrics_circle_hetero_mov_sen_mov_tar_08-Feb-2017'];
load(filename);

% remember: use the mean values of ml error and entropy, i.e.,
% ml_err_xxx_mean, ent_xxx_mean
var_name_set = {'ml_err_dbf_mean','ent_dbf_mean'};
% var_name_set = {'ml_err_cons_mean','ent_cons_mean'};
% var_name_set = {'ml_err_cent_mean','ent_cent_mean'};

for ii = 1:length(var_name_set)
    csvwrite([filepath,var_name_set{ii}],sim_for_save.sim_res.(var_name_set{ii}))
end
%}

%% test if the analysis of trim time is correct or not
%{
% define four topologies used in simulation
% so the way I use the adjacency matrix to analyze seems incorrect.
% revisit later.
A = {[0 1 0 0 0 0; 1 0 1 0 0 0; 0 1 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0]+eye(6),...
    [0 0 0 0 0 1; 0 0 0 0 0 0; 0 0 0 1 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0]+eye(6),...
    [0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 1 0; 0 0 0 0 0 0; 0 0 0 0 0 0]+eye(6),...
    [0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 1 0 0 0; 0 0 1 0 0 0]+eye(6)};
topo_set = repmat([1 2 2 2 4 1 3 1 3 4],1,5);

l_set = [];
j_set = [];
t_set = [];

n = 20;
for t = 1:n-1
    B1 = eye(6);
    for t2 = 1:t
        B1 = B1*A{topo_set(t2)};
    end
    for ll = 1:6
        for jj = 2:6
            if ll == jj
                continue
            end
            if B1(ll,jj) > 0
                B2 = eye(6);
                for t3 = t+1:n
                    B2 = B2*A{topo_set(t3)};
                end
                if B2(jj,1) > 0
%                     display('connected paths found');
                    l_set = [l_set,ll];
                    j_set = [j_set,jj];
                    t_set = [t_set,t];
                end
            end
        end
    end
end
%}

%% change rbt_spec
%{
tmp = [[35;73],[50;77],[21;26],[75;15],[46;22],[77;58]];
for ii = 1:6
    rbt(ii).init_pos = [rbt(ii).init_pos,tmp(:,ii)];
end
save('rbt_spec.mat','rbt')
%}