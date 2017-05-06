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

%% compute metric of all test trials for dbf-pf. 
% This is a temporary but important code. 
% The code has been incorporated into compareMetricsPF in
% Sim class. But I keep the original copy here for future reference

load('./figures/JDSMC/metrics_plot/metrics_circle_hetero_mov_sen_mov_tar_01-May-2017','sim_for_save');

tmp_sim_res = []; %this.sim_res;
for jj = 1:10
    for ii = 1:6
        % ml error
        tmp_sim_res.ml_err_dbf_pf(ii,jj,:) = sim.rbt_set{jj}.rbt{ii}.ml_err_dbf_pf; 
        % norm of cov of pdf
%         tmp_sim_res.pdf_norm_dbf_pf(ii,jj,:) = sim.rbt_set{jj}.rbt{ii}.pdf_norm_dbf_pf;
        % entropy of pdf
        tmp_sim_res.ent_dbf_pf(ii,jj,:) = sim.rbt_set{jj}.rbt{ii}.ent_dbf_pf;
    end
end

for ii = 1:6
    % ml error
    % dbf-pf
    tmp_ml_err_dbf_pf = squeeze(tmp_sim_res.ml_err_dbf_pf(ii,:,:));
    tmp_sim_res.ml_err_dbf_pf_mean(ii,:) = mean(tmp_ml_err_dbf_pf,1);
    tmp_sim_res.ml_err_dbf_pf_cov(ii,:) = diag(cov(tmp_ml_err_dbf_pf))';
       
%     % norm of cov of pdf
%     tmp_pdf_norm_dbf_pf = squeeze(tmp_sim_res.pdf_norm_dbf_pf(ii,:,:));
%     tmp_sim_res.pdf_norm_dbf_pf_mean(ii,:) = mean(tmp_pdf_norm_dbf_pf,1);
%     tmp_sim_res.pdf_norm_dbf_pf_cov(ii,:) = diag(cov(tmp_pdf_norm_dbf_pf)');
    
    % entropy of pdf
    % dbf
    tmp_ent_dbf_pf = squeeze(tmp_sim_res.ent_dbf_pf(ii,:,:));
    tmp_sim_res.ent_dbf_pf_mean(ii,:) = mean(tmp_ent_dbf_pf,1);
    tmp_sim_res.ent_dbf_pf_cov(ii,:) = diag(cov(tmp_ent_dbf_pf)');    
end

sim_for_save.sim_res = tmp_sim_res;
save('./figures/JDSMC/metrics_plot/metrics_circle_hetero_mov_sen_mov_tar_01-May-2017-2','sim_for_save')

%% %%%%%%%%%%%%%% plot the performance metrics %%%%%%%%%%%%%%%%%
hf_err = figure(1);
line_clr = ['r','g','b','c','m','k'];
line_marker = {'o','*','s','d','^','h'};
count = 50;
% for LIFO-DBF, we draw different robot's performance metrics
for ii = [1,3,5]
    plot(1:count-2,tmp_sim_res.ml_err_dbf_pf_mean(ii,1:count-2),line_clr(ii),'LineWidth',2,'Marker',line_marker{ii},'MarkerSize',2); hold on;
end
xlim([0,count-1])

% add legend
[~, hobj1] = legend('DBF-R1','DBF-R3','DBF-R5');
textobj = findobj(hobj1, 'type', 'text');
set(textobj, 'fontsize', 15);

title('Target Position Error','FontSize',30);
set(gca,'fontsize',30)
xlabel('Time (Step)','FontSize',30);
ylabel('Position Error','FontSize',30);

% entropy
hf_ent = figure(2);
line_clr = ['r','g','b','c','m','k'];
line_marker = {'o','*','s','d','^','h'};
for ii=[1,3,5]%plot_rbt_idx
    plot(1:count-2,tmp_sim_res.ent_dbf_pf_mean(ii,1:count-2),line_clr(ii),'LineWidth',2,'Marker',line_marker{ii},'MarkerSize',2); hold on;
end
xlim([0,count-1])

% add legend
[~, hobj3] = legend('DBF-R1','DBF-R3','DBF-R5');
textobj = findobj(hobj3, 'type', 'text');
set(textobj, 'fontsize', 15);

title('Entropy of the Target PDF','FontSize',30);
set(gca,'fontsize',30)
xlabel('Time (Step)','FontSize',30);
ylabel('Entropy','FontSize',30);