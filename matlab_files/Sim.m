classdef Sim
    properties
        dt; % discretization time interval
        r_move;
        tar_move;
        sim_len;
        cons_step;
        num_robot;
        r_init_pos_set;
        sim_r_idx;
        trial_num;
        selection;
        rbt_set; % record all rbt objects
        sim_res; % records the metrics of different filtering methods
    end
    
    methods
        
        function this = Sim(inPara)
            this.dt = inPara.dt;
            this.r_move = inPara.r_move;
            this.tar_move = inPara.tar_move;
            this.sim_len = inPara.sim_len;
            this.cons_step = inPara.cons_step;
            this.num_robot = inPara.num_robot;
            this.r_init_pos_set = inPara.r_init_pos_set;
            this.sim_r_idx = inPara.sim_r_idx;
            this.trial_num = inPara.trial_num;
        end
        
        function saveSimData(this)
            % save the simulation data, mainly the Robot and Field objects
            switch this.selection
                case 1,  tag = 'sta_sen_sta_tar';
                case 2,  tag = 'sta_sen_mov_tar';
                case 3,  tag = 'mov_sen_sta_tar';
                case 4,  tag = 'mov_sen_mov_tar';
            end
            
            file_name = sprintf('./figures/data_exchange/Journal/%s_robot_%s.mat',tag,datestr(now,1));
            
            %%%%% need to revise. may not want to save upd_cell %%%%%
            save(file_name) % save current workspace
        end
        
        function plotSim(this,rbt,fld,count)
            % Plotting for simulation process
            fig_cnt = 1;
            %% LIFO-DBF
            %
            % plot figures for selected robots
            for k = this.sim_r_idx
                fig_cnt = fig_cnt+1;
                tmp_hd = figure (fig_cnt); % handle for plot of a single robot's target PDF
                clf(tmp_hd);
                shading interp
                contourf((rbt{k}.dbf_map)','LineColor','none');
                load('MyColorMap','mymap')
                colormap(mymap);
                colorbar
                hold on;
                for j=1:this.num_robot
                    % draw robot trajectory
                    if j==k
                        line_hdl = line(rbt{j}.traj(1,:), rbt{j}.traj(2,:));
                        set(line_hdl,'Marker','.','Color','r','MarkerSize',3,'LineWidth',2);
                        plot(rbt{j}.traj(1,end), rbt{j}.traj(2,end), 's','Color','r','MarkerSize',25,'LineWidth',3);
                    else
                        line_hdl = line(rbt{j}.traj(1,:), rbt{j}.traj(2,:));
                        set(line_hdl,'Marker','.','Color','g','MarkerSize',3,'LineWidth',2);
                        plot(rbt{j}.traj(1,end), rbt{j}.traj(2,end), 'p','Color','g','MarkerSize',25,'LineWidth',1.5);
                    end
                    
                    % draw traget trajectory
                    line_hdl = line(fld.target.traj(1,:), fld.target.traj(2,:));
                    set(line_hdl,'Marker','.','Color','k','MarkerSize',3,'LineWidth',2);
                    plot(fld.target.pos(1), fld.target.pos(2), 'k+','MarkerSize',25,'LineWidth',3);
                    set(gca,'fontsize',30)
                end
                xlabel(['Step=',num2str(count)],'FontSize',30);
            end
            %}
            
            %% Consensus
            % plot figures for selected robots
            %{
            for k = 1%sim_r_idx
                fig_cnt = fig_cnt+1;
                tmp_hd = figure (fig_cnt); % handle for plot of a single robot's target PDF
                clf(tmp_hd);
                shading interp
                contourf((rbt_cons(k).map)','LineColor','none');
                load('MyColorMap','mymap')
                colormap(mymap);
                colorbar
                hold on;
                for j=1:NumOfRobot
                    
                    % draw robot trajectory
                    if j==k
                        line_hdl = line(rbt(j).traj(1,:), rbt(j).traj(2,:));
                        set(line_hdl,'Marker','.','Color','r','MarkerSize',3,'LineWidth',2);
                        plot(rbt(j).traj(1,end), rbt(j).traj(2,end), 's','Color','r','MarkerSize',25,'LineWidth',3);
                    else
                        line_hdl = line(rbt(j).traj(1,:), rbt(j).traj(2,:));
                        set(line_hdl,'Marker','.','Color','g','MarkerSize',3,'LineWidth',2);
                        plot(rbt(j).traj(1,end), rbt(j).traj(2,end), 'p','Color','g','MarkerSize',25,'LineWidth',1.5);
                    end
                    
                    % draw target trajectory
                    line_hdl = line(fld.traj(1,:), fld.traj(2,:));
                    set(line_hdl,'Marker','.','Color','k','MarkerSize',3,'LineWidth',2);
                    plot(fld.tx, fld.ty, 'k+','MarkerSize',25,'LineWidth',3);
                    set(gca,'fontsize',30)
                end
                xlabel(['Step=',num2str(count)],'FontSize',30);
            end
            %}
            
            %% Centralized
            %{
            % plot figures for central map
            fig_cnt = fig_cnt+1;
            tmp_hd = figure (fig_cnt); % handle for plot of a single robot's target PDF
            clf(tmp_hd);
            shading interp
            contourf((rbt_cent.map)','LineColor','none');
            load('MyColorMap','mymap')
            colormap(mymap);
            colorbar
            xlabel(['Step=',num2str(count)],'FontSize',16);
            
            hold on;
            
            % draw robot trajectory
            for j=1:NumOfRobot
                line_hdl = line(rbt(j).traj(1,:), rbt(j).traj(2,:));
                set(line_hdl,'Marker','.','Color','g','MarkerSize',3,'LineWidth',2);
                plot(rbt(j).traj(1,end), rbt(j).traj(2,end), 'p','Color','g','MarkerSize',25,'LineWidth',1.5);
            end
            
            % draw target trajectory
            line_hdl = line(fld.traj(1,:), fld.traj(2,:));
            set(line_hdl,'Marker','.','Color','k','MarkerSize',3,'LineWidth',2);
            plot(fld.tx, fld.ty, 'k+','MarkerSize',25,'LineWidth',3);
            set(gca,'fontsize',30)
            %}
            
            % save plots
            %{
        if (count == 1) || (count == 3) || (count == 5) || (count == 7) ||...
                (count == 10) || (count == 20) || (count == 30) || (count == 40)...
                || (count == 50) || (count == 60) || (count == 70) || (count == 80)...
                || (count == 90) || (count == 100)
            switch Selection2
                case 1,  tag = 'sta_sen_sta_tar';
                case 2,  tag = 'sta_sen_mov_tar';
                case 3,  tag = 'mov_sen_sta_tar';
                case 4,  tag = 'mov_sen_mov_tar';
            end
            %         file_name1 = sprintf('./figures/data_exchange_switch/%s_%d_%s',tag,count,datestr(now,1));
            %         saveas(hf1,file_name1,'fig')
            %         saveas(hf1,file_name1,'jpg')
            for k = sim_r_idx
                tmp_hf = figure(k+2);
                file_name2 = sprintf('./figures/data_exchange/%s_single_%d_%d_%s',tag,k,count,datestr(now,1));
                if save_file == 1
                    saveas(tmp_hf,file_name2,'fig')
                    saveas(tmp_hf,file_name2,'jpg')
                end
            end
        end
            %}
            
        end
        
        function this = compareMetrics(this)
            for jj = 1:this.trial_num
                for ii = 1:this.num_robot
                    % ml error
                    sim_res.ml_err_dbf(ii,jj,:) = this.rbt_set{jj}.rbt{ii}.ml_err_dbf;
                    sim_res.ml_err_cons(ii,jj,:) = this.rbt_set{jj}.rbt{ii}.ml_err_cons;
                    
                    % norm of cov of pdf
                    sim_res.pdf_norm_dbf(ii,jj,:) = this.rbt_set{jj}.rbt{ii}.pdf_norm_dbf;
                    sim_res.pdf_norm_cons(ii,jj,:) = this.rbt_set{jj}.rbt{ii}.pdf_norm_cons;
                    
                    % entropy of pdf
                    sim_res.ent_dbf(ii,jj,:) = this.rbt_set{jj}.rbt{ii}.ent_dbf;
                    sim_res.ent_cons(ii,jj,:) = this.rbt_set{jj}.rbt{ii}.ent_cons;
                end
                sim_res.ml_err_cent(jj,:) = this.rbt_set{jj}.rbt{1}.ml_err_cent;
                sim_res.pdf_norm_cent(jj,:) = this.rbt_set{jj}.rbt{1}.rbt_cent.pdf_norm_cent;
                sim_res.ent_cent(jj,:) = this.rbt_set{jj}.rbt{1}.rbt_cent.ent_cent;
            end
            
            for ii = 1:this.num_robot
                % ml error
                % dbf
                tmp_ml_err_dbf = squeeze(sim_res.ml_err_dbf(ii,:,:));
                sim_res.ml_err_dbf_mean(ii,:) = mean(tmp_ml_err_dbf,1);
                sim_res.ml_err_dbf_cov(ii,:) = diag(cov(tmp_ml_err_dbf))';
                
                % consensus
                tmp_ml_err_cons = squeeze(sim_res.ml_err_cons(ii,:,:));
                sim_res.ml_err_cons_mean(ii,:) = mean(tmp_ml_err_cons,1);
                sim_res.ml_err_cons_cov(ii,:) = diag(cov(tmp_ml_err_cons)');
                
                % norm of cov of pdf
                % dbf
                tmp_pdf_norm_dbf = squeeze(sim_res.pdf_norm_dbf(ii,:,:));
                sim_res.pdf_norm_dbf_mean(ii,:) = mean(tmp_pdf_norm_dbf,1);
                sim_res.pdf_norm_dbf_cov(ii,:) = diag(cov(tmp_pdf_norm_dbf)');
                
                % consensus
                tmp_pdf_norm_cons = squeeze(sim_res.pdf_norm_cons(ii,:,:));
                sim_res.pdf_norm_cons_mean(ii,:) = mean(tmp_pdf_norm_cons,1);
                sim_res.pdf_norm_cons_cov(ii,:) = diag(cov(tmp_pdf_norm_cons)');
                
                % entropy of pdf
                % dbf
                tmp_entropy_dbf = squeeze(sim_res.entropy_dbf(ii,:,:));
                sim_res.ent_dbf_mean(ii,:) = mean(tmp_entropy_dbf,1);
                sim_res.ent_dbf_cov(ii,:) = diag(cov(tmp_entropy_dbf)');
                
                % consensus
                tmp_entropy_cons = squeeze(sim_res.entropy_cons(ii,:,:));
                sim_res.ent_cons_mean(ii,:) = mean(tmp_entropy_cons,1);
                sim_res.ent_cons_cov(ii,:) = diag(cov(tmp_entropy_cons)');
            end
            
            % ml error
            % centralized
            tmp_ml_err_cent = sim_res.ml_err_cent;
            sim_res.ml_err_cent_mean = mean(tmp_ml_err_cent,1);
            sim_res.ml_err_cent_cov = diag(cov(tmp_ml_err_cent)');
            
            % norm of cov of pdf
            % centralized
            tmp_pdf_norm_cent = sim_res.pdf_norm_cent;
            sim_res.pdf_norm_cent_mean = mean(tmp_pdf_norm_cent,1);
            sim_res.pdf_norm_cent_cov = diag(cov(tmp_pdf_norm_cent)');
            
            % entropy of pdf
            % centralized
            tmp_ent_cent = sim_res.entropy_cent;
            sim_res.ent_cent_mean = mean(tmp_ent_cent,1);
            sim_res.ent_cent_cov = diag(cov(tmp_ent_cent)');
            
            %% %%%%%%%%%%%%%% plot the performance metrics %%%%%%%%%%%%%%%%%
            plot_rbt_idx = 1:2:5; % draw robot 1, 3, 5
            fig_cnt = 1;
            % ml error
            fig_cnt = fig_cnt+1;
            hf_err = figure(fig_cnt);
            line_clr = ['r','g','b','c','m','k'];
            line_marker = {'o','*','s','d','^','h'};
            
            % for LIFO-DBF, we draw different robot's performance metrics
            for i=plot_rbt_idx
                plot(1:count-2,sim_res.ml_err_dbf_mean(i,1:count-2),line_clr(i),'LineWidth',2,'Marker',line_marker{i},'MarkerSize',2); hold on;
                % errorbar(1:count-2,sim_res.ml_err_dbf_mean(i,1:count-2),sqrt(sim_res.ml_err_dbf_cov(i,1:count-2)),...
                %         line_clr(i),'LineWidth',2,'Marker',line_marker{i},'MarkerSize',2); hold on;
            end
            
            % for consensus, we draw one robot's performance metrics.
            
            % for i=1:NumOfRobot
            %     plot(1:count-2,rbt_cons(i).ml_err_cons(1:count-2),line_clr(i),'LineStyle','--','LineWidth',2,'Marker',line_marker{i},'MarkerSize',2); hold on;
            % end
            plot(1:count-2,sim_res.ml_err_cons_mean(1,1:count-2),line_clr(2),'LineStyle','--','LineWidth',2,'Marker',line_marker{2},'MarkerSize',2); hold on;
            % errorbar(1:count-2,sim_res.ml_err_cons_mean(1,1:count-2),sqrt(sim_res.ml_err_cons_cov(1,1:count-2)),...
            %     line_clr(1),'LineStyle','--','LineWidth',2,'Marker',line_marker{1},'MarkerSize',2); hold on;
            
            % only on centralized filter
            plot(1:count-2,sim_res.ml_err_cent_mean(1:count-2),line_clr(6),'LineStyle','-.','LineWidth',2,'Marker',line_marker{6},'MarkerSize',2); hold on;
            % errorbar(1:count-2,sim_res.ml_err_cent_mean(1:count-2),sqrt(sim_res.ml_err_cent_cov(1:count-2)),...
            %     line_clr(6),'LineStyle','-.','LineWidth',2,'Marker',line_marker{6},'MarkerSize',2); hold on;
            xlim([0,count-1])
            
            % add legend
            [~, hobj1] = legend('DBF-R1','DBF-R3','DBF-R5','Consen','Central');
            textobj = findobj(hobj1, 'type', 'text');
            set(textobj, 'fontsize', 15);
            
            title('Target Position Error','FontSize',30);
            set(gca,'fontsize',30)
            xlabel('Time','FontSize',30);
            ylabel('Position Error','FontSize',30);
            
            %{
                switch Selection2
                    case 1,  tag = 'sta_sen_sta_tar';
                    case 2,  tag = 'sta_sen_mov_tar';
                    case 3,  tag = 'mov_sen_sta_tar';
                    case 4,  tag = 'mov_sen_mov_tar';
                end
                file_name2 = sprintf('./figures/data_exchange/%s_entropy_%s',tag,datestr(now,1));
                if save_file == 1
                    saveas(hf_err,file_name2,'fig')
                    saveas(hf_err,file_name2,'jpg')
                end
            %}
            
            % pdf covariance norm
            % results are hard to interpret. So just not include this figure
            %{
                fig_cnt = fig_cnt+1;
                hf_cov = figure(fig_cnt);
                line_clr = ['r','g','b','c','m','k'];
                line_marker = {'o','*','s','d','^','h'};
                for i=plot_rbt_idx
                %     plot(1:count-2,rbt(i).pdf_norm_dbf(1:count-2),line_clr(i),'LineWidth',2,'Marker',line_marker{i},'MarkerSize',2); hold on;
                    plot(1:count-2,sim_res.pdf_norm_dbf_mean(i,1:count-2),line_clr(i),'LineWidth',2,'Marker',line_marker{i},'MarkerSize',2); hold on;
                %     errorbar(1:count-2,sim_res.pdf_norm_dbf_mean(i,1:count-2),sqrt(sim_res.pdf_norm_dbf_cov(i,1:count-2)),...
                %         line_clr(i),'LineWidth',2,'Marker',line_marker{i},'MarkerSize',2); hold on;
                end
                [~, hobj2] = legend('Robot 1','Robot 2','Robot 3','Robot 4','Robot 5','Robot 6');
                textobj = findobj(hobj2, 'type', 'text');
                set(textobj, 'fontsize', 24);

                % plot(1:count-2,rbt_cons(i).pdf_norm_cons(1:count-2),line_clr(i),'LineStyle','--','LineWidth',2,'Marker',line_marker{i},'MarkerSize',2); hold on;
                plot(1:count-2,sim_res.pdf_norm_cons_mean(1,1:count-2),line_clr(2),'LineStyle','--','LineWidth',2,'Marker',line_marker{i},'MarkerSize',2); hold on;
                % errorbar(1:count-2,sim_res.pdf_norm_cons_mean(1,1:count-2),sqrt(sim_res.pdf_norm_cons_cov(1,1:count-2)),...
                %     line_clr(1),'LineStyle','--','LineWidth',2,'Marker',line_marker{i},'MarkerSize',2); hold on;

                % plot(1:count-2,rbt_cent.pdf_norm_cent(1:count-2),line_clr(i),'LineStyle','-.','LineWidth',2,'Marker',line_marker{i},'MarkerSize',2); hold on;
                plot(1:count-2,sim_res.pdf_norm_cent_mean(1:count-2),line_clr(6),'LineStyle','-.','LineWidth',2,'Marker',line_marker{i},'MarkerSize',2); hold on;
                % errorbar(1:count-2,sim_res.pdf_norm_cent_mean(1:count-2),sqrt(sim_res.pdf_norm_cent_cov(1:count-2)),...
                %     line_clr(6),'LineStyle','-.','LineWidth',2,'Marker',line_marker{i},'MarkerSize',2); hold on;
                xlim([0,count-1])

                title('Covariance of Target PDF','FontSize',30);
                set(gca,'fontsize',30)
                xlabel('Time','FontSize',30);
                ylabel('Norm of Covariance Matrix','FontSize',30);



            %{
                switch Selection2
                    case 1,  tag = 'sta_sen_sta_tar';
                    case 2,  tag = 'sta_sen_mov_tar';
                    case 3,  tag = 'mov_sen_sta_tar';
                    case 4,  tag = 'mov_sen_mov_tar';
                end
                file_name2 = sprintf('./figures/data_exchange/%s_entropy_%s',tag,datestr(now,1));
                if save_file == 1
                    saveas(hf_cov,file_name2,'fig')
                    saveas(hf_cov,file_name2,'jpg')
                end
            %}
            %}
            
            % entropy
            fig_cnt = fig_cnt+1;
            hf_ent = figure(fig_cnt);
            line_clr = ['r','g','b','c','m','k'];
            line_marker = {'o','*','s','d','^','h'};
            for i=plot_rbt_idx
                %     plot(1:count-2,rbt(i).entropy(1:count-2),line_clr(i),'LineWidth',2,'Marker',line_marker{i},'MarkerSize',2); hold on;
                plot(1:count-2,sim_res.entropy_dbf_mean(i,1:count-2),line_clr(i),'LineWidth',2,'Marker',line_marker{i},'MarkerSize',2); hold on;
                %     errorbar(1:count-2,sim_res.entropy_dbf_mean(i,1:count-2),sqrt(sim_res.entropy_dbf_cov(i,1:count-2)),line_clr(i),'LineWidth',2,'Marker',line_marker{i},'MarkerSize',2); hold on;
            end
            
            % plot(1:count-2,rbt_cons(i).entropy(1:count-2),line_clr(i),'LineStyle','--','LineWidth',2,'Marker',line_marker{i},'MarkerSize',2); hold on;
            plot(1:count-2,sim_res.entropy_cons_mean(1,1:count-2),line_clr(2),'LineStyle','--','LineWidth',2,'Marker',line_marker{2},'MarkerSize',2); hold on;
            % errorbar(1:count-2,sim_res.entropy_cons_mean(1,1:count-2),sqrt(sim_res.entropy_cons_cov(1,1:count-2)),line_clr(1),'LineStyle','--','LineWidth',2,'Marker',line_marker{i},'MarkerSize',2); hold on;
            
            % plot(1:count-2,rbt_cent.entropy(1:count-2),line_clr(i),'LineStyle','-.','LineWidth',2,'Marker',line_marker{i},'MarkerSize',2); hold on;
            plot(1:count-2,sim_res.entropy_cent_mean(1:count-2),line_clr(6),'LineStyle','-.','LineWidth',2,'Marker',line_marker{6},'MarkerSize',2); hold on;
            % errorbar(1:count-2,sim_res.entropy_cent_mean(1:count-2),sqrt(sim_res.entropy_cent_cov(1:count-2)),line_clr(6),'LineStyle','-.','LineWidth',2,'Marker',line_marker{i},'MarkerSize',2); hold on;
            xlim([0,count-1])
            
            % add legend
            [~, hobj3] = legend('DBF-R1','DBF-R3','DBF-R5','Consen','Central');
            textobj = findobj(hobj3, 'type', 'text');
            set(textobj, 'fontsize', 15);
            
            title('Entropy of the Target PDF','FontSize',30);
            set(gca,'fontsize',30)
            xlabel('Time','FontSize',30);
            ylabel('Entropy','FontSize',30);
            
            %{
                switch Selection2
                    case 1,  tag = 'sta_sen_sta_tar';
                    case 2,  tag = 'sta_sen_mov_tar';
                    case 3,  tag = 'mov_sen_sta_tar';
                    case 4,  tag = 'mov_sen_mov_tar';
                end
                file_name2 = sprintf('./figures/data_exchange/%s_entropy_%s',tag,datestr(now,1));
                if save_file == 1
                    saveas(hf_ent,file_name2,'fig')
                    saveas(hf_ent,file_name2,'jpg')
                end
            %}
            this.sim_res = sim_res;
        end
        
    end
end