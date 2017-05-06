% this file is for generating comparison plot for JDSMC 1st round revision
% it compares the results of DBF-PF and the results of the DBF (using 
% histogram filter) in the original submission. Rigorously speaking, they are different since the
% sensor measurements are different (not the same set of measurement data).
% But are similar qualitatively

% choose the target model
tar_model = 'circle';
%load previous data (DBF-HF)
load('./figures/JDSMC/metrics_plot/metrics_linear_hetero_mov_sen_mov_tar_09-Feb-2017','sim_for_save');
sim_lin_dbf = sim_for_save;
load('./figures/JDSMC/metrics_plot/metrics_sin_hetero_mov_sen_mov_tar_09-Feb-2017','sim_for_save');
sim_sin_dbf = sim_for_save;
load('./figures/JDSMC/metrics_plot/metrics_circle_hetero_mov_sen_mov_tar_08-Feb-2017','sim_for_save');
sim_cir_dbf = sim_for_save;

% load new data (DBF-PF)
load('./figures/JDSMC/metrics_plot/metrics_linear_hetero_mov_sen_mov_tar_01-May-2017-2','sim_for_save');
sim_lin_dbf_pf = sim_for_save;
load('./figures/JDSMC/metrics_plot/metrics_sin_hetero_mov_sen_mov_tar_01-May-2017','sim_for_save');
sim_sin_dbf_pf = sim_for_save;
load('./figures/JDSMC/metrics_plot/metrics_circle_hetero_mov_sen_mov_tar_01-May-2017-2','sim_for_save');
sim_cir_dbf_pf = sim_for_save;

switch tar_model
    case 'linear'
        tar_idx = 1;
        sim_dbf = sim_lin_dbf;
        sim_dbf_pf = sim_lin_dbf_pf;
    case 'sin'
        tar_idx = 2;
        sim_dbf = sim_sin_dbf;
        sim_dbf_pf = sim_sin_dbf_pf;
    case 'circle'
        tar_idx = 3;
        sim_dbf = sim_cir_dbf;
        sim_dbf_pf = sim_cir_dbf_pf;
end

%% %%%%%%%%%%%%%% plot the performance metrics %%%%%%%%%%%%%%%%%
hf_err = figure(1);
line_clr = ['r','g','b','c','m','k'];
line_marker = {'o','*','s','d','^','h'};
count = 50;
% for LIFO-DBF, we draw different robot's performance metrics
for ii = 1:6
    plot(1:count-2,sim_dbf.sim_res.ml_err_dbf_mean(ii,1:count-2),line_clr(ii),'LineWidth',1,'Marker',line_marker{ii},'MarkerSize',1); hold on;  
end
for ii = 1:6
    plot(1:count-2,sim_dbf_pf.sim_res.ml_err_dbf_pf_mean(ii,1:count-2),line_clr(ii),'LineWidth',1,'Marker',line_marker{ii},'LineStyle','--','MarkerSize',1);hold on; 
end
xlim([0,count-1])

% add legend
lenged_str = {'HF-R1','HF-R2','HF-R3','HF-R4','HF-R5','HF-R6','PF-R1','PF-R2','PF-R3','PF-R4','PF-R5','PF-R6'};
[~, hobj1] = columnlegend(2,lenged_str,'northeast');
textobj = findobj(hobj1, 'type', 'text');
set(textobj, 'fontsize', 14);

title(sprintf('Position Error of Target %d',tar_idx),'FontSize',30);
set(gca,'fontsize',30)
xlabel('Time (Step)','FontSize',30);
ylabel('Position Error','FontSize',30);

% entropy
hf_ent = figure(2);
line_clr = ['r','g','b','c','m','k'];
line_marker = {'o','*','s','d','^','h'};
for ii=1:6
    plot(1:count-2,sim_dbf.sim_res.ent_dbf_mean(ii,1:count-2),line_clr(ii),'LineWidth',1,'Marker',line_marker{ii},'MarkerSize',1);hold on;    
end
for ii = 1:6
   plot(1:count-2,sim_dbf_pf.sim_res.ent_dbf_pf_mean(ii,1:count-2),line_clr(ii),'LineWidth',1,'Marker',line_marker{ii},'LineStyle','--','MarkerSize',1);hold on; 
end
xlim([0,count-1])

% add legend
lenged_str = {'HF-R1','HF-R2','HF-R3','HF-R4','HF-R5','HF-R6','PF-R1','PF-R2','PF-R3','PF-R4','PF-R5','PF-R6'};
[~, hobj3] = columnlegend(2,lenged_str,'northeast');
textobj = findobj(hobj3, 'type', 'text');
set(textobj, 'fontsize', 14);

title(sprintf('Entropy of Target %d',tar_idx),'FontSize',30);
set(gca,'fontsize',30)
xlabel('Time (Step)','FontSize',30);
ylabel('Entropy','FontSize',30);