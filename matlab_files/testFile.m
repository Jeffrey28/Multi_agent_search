%% convert sim class to usually struct
% sim_for_save2 = [];
% fldnames = fieldnames(sim.sim_res);
% for ii = 1:length(fldnames)
%     sim_for_save2.(fldnames{ii}) = sim.sim_res.(fldnames{ii});
% end
% save('test_mat','sim_for_save2')

%% write data to cvs, which will be then copied for matplotlib plotting
filepath = './figures/data_exchange/Journal/';
filename = [filepath,'metrics_hetero_mov_sen_mov_tar_11-Oct-2016'];
load(filename);

var_name_set = {'ml_err_dbf_mean','ml_err_cons_mean','ml_err_cent_mean',...
    'ent_dbf_mean','ent_cons_mean','ent_cent_mean'};

for ii = 1:length(var_name_set)
    csvwrite([filepath,var_name_set{ii}],sim_for_save.sim_res.(var_name_set{ii}))
end