classdef Sim
    properties
        dt; % discretization time interval
        r_move;
        tar_move;
        sim_len;
        cons_step;
        cons_fig;
        num_robot;
        r_init_pos_set;
        fld_size;
        t_init_pos_set;
        sim_r_idx;
        trial_num;
        selection;
        rbt_set; % record all rbt objects
        fld_set; % record all field objects
        sim_res; % records the metrics of different filtering methods
        fig_cnt; % counter for figure
        sensor_set_type; % the type of robot team        
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
            this.t_init_pos_set = inPara.t_init_pos_set;
            this.fld_size = inPara.fld_size;
            this.sim_r_idx = inPara.sim_r_idx;
            this.trial_num = inPara.trial_num;
            this.fig_cnt = 1;
            this.sensor_set_type = inPara.sensor_set_type;
            this.selection = inPara.selection;
        end
       
        
        function dbf_hd = plotSim(this,rbt,fld,count,save_plot)
            % Plotting for simulation process
            tmp_fig_cnt = this.fig_cnt;
            %% LIFO-DBF
            %
            % plot figures for selected robots
            for k = this.sim_r_idx
                dbf_hd = figure (tmp_fig_cnt); % handle for plot of a single robot's target PDF
                clf(dbf_hd);
                shading interp
                contourf((rbt{k}.dbf_map)','LineColor','none');
                load('MyColorMap','mymap')
                colormap(mymap);
                colorbar
                hold on;
                for j=1:this.num_robot
                    % draw robot trajectory
                    if j==k
                        % draw the robot's whole trajectory
%                         line_hdl = line(rbt{j}.traj(1,:), rbt{j}.traj(2,:));
%                         set(line_hdl,'Marker','.','Color','r','MarkerSize',3,'LineWidth',2);
%                         plot(rbt{j}.traj(1,end), rbt{j}.traj(2,end), 's','Color','r','MarkerSize',25,'LineWidth',3);
                        % only draw robot's current position. used in the
                        % progress video.
                        plot(rbt{j}.traj(1,end), rbt{j}.traj(2,end), 's','Color','r','MarkerSize',20,'MarkerFaceColor','r','LineWidth',1);
                    else
                        % draw the robot's whole trajectory
%                         line_hdl = line(rbt{j}.traj(1,:), rbt{j}.traj(2,:));
%                         set(line_hdl,'Marker','.','Color','g','MarkerSize',3,'LineWidth',2);
%                         plot(rbt{j}.traj(1,end), rbt{j}.traj(2,end), 'p','Color','g','MarkerSize',25,'LineWidth',1.5);
                        % only draw robot's current position. used in the
                        % progress video.
                        plot(rbt{j}.traj(1,end), rbt{j}.traj(2,end), 's','Color','g','MarkerSize',20,'MarkerFaceColor','g','LineWidth',1);
                    end
                    text(rbt{j}.traj(1,end)-3, rbt{j}.traj(2,end)-3, sprintf('%d',j),'FontSize',10);
                    
                    % draw target trajectory
                    line_hdl = line(fld.target.traj(1,:), fld.target.traj(2,:));
                    set(line_hdl,'Marker','.','Color','k','MarkerSize',3,'LineWidth',2);
%                     plot(fld.target.pos(1), fld.target.pos(2), 'k+','MarkerSize',25,'LineWidth',3);
                    plot(fld.target.pos(1), fld.target.pos(2), 'kp','MarkerSize',20,'MarkerFaceColor','k','LineWidth',1);
                    plot(fld.target.traj(1,:), fld.target.traj(2,:), 'LineWidth',3);
                    set(gca,'fontsize',30)                    
                end
                title(sprintf('DBF Robot %d',k))
                xlabel(['Step=',num2str(count)],'FontSize',30);
                tmp_fig_cnt = tmp_fig_cnt+1;
                
                drawnow
                
                % save figure      
                if save_plot
                    if (count == 1) || (count == 3) || (count == 7) || (count == 10) ...
                            || (count == 20) || (count == 30) || (count == 40)...
                            || (count == 5) || (count == 45) %|| (count == 10) || (count == 20) || (count == 30) 
                        switch this.selection
                            case 1,  tag = 'sta_sen_sta_tar';
                            case 2,  tag = 'sta_sen_mov_tar';
                            case 3,  tag = 'mov_sen_sta_tar';
                            case 4,  tag = 'mov_sen_mov_tar';
                        end
                        
                        switch this.sensor_set_type
                            case 'brg', tag2 = 'brg';
                            case 'ran', tag2 = 'ran';
                            case 'rb', tag2 = 'rb';
                            case 'htr', tag2 = 'hetero';
                            case 'sonar', tag2 = 'sonar';
                        end
                        
                        % save the plot
                        file_name2 = sprintf('./figures/data_exchange/Journal/process_plot/%s_%s_rbt%d_step%d_%s',...
                            tag2,tag,k,count,datestr(now,1));
                        saveas(dbf_hd,file_name2,'fig')
                        saveas(dbf_hd,file_name2,'jpg')
                        
                        % save data of prob map
                        file_name_map = sprintf('./figures/data_exchange/Journal/process_plot/%s_%s_map_rbt%d_step%d_%s',...
                            tag2,tag,k,count,datestr(now,1));
                        
                        tmp_map = rbt{k}.dbf_map;
                        save(file_name_map,'tmp_map');
                        
                        % save all robot's trajectory when k is the last
                        % element of this.sim_r_idx
                        if k == this.sim_r_idx(end)
                            file_name_tar_traj = sprintf('./figures/data_exchange/Journal/process_plot/%s_%s_tar_traj_%s',...
                                    tag2,tag,datestr(now,1));
                            tmp_tar_traj = fld.target.traj;
                            save(file_name_tar_traj,'tmp_tar_traj');
                            for j = 1:this.num_robot
                                file_name_rbt_traj = sprintf('./figures/data_exchange/Journal/process_plot/%s_%s_rbt%d_traj_%s',...
                                    tag2,tag,j,datestr(now,1));
                                tmp_rbt_traj = rbt{j}.traj;
                                save(file_name_rbt_traj,'tmp_rbt_traj');
                            end
                        end
                    end
                end
            end
            %}
            
            %% Consensus
            % plot figures for selected robots
            %{
            for k = 1:this.sim_r_idx                
                tmp_hd = figure (tmp_fig_cnt); % handle for plot of a single robot's target PDF
                clf(tmp_hd);
                shading interp
                contourf((rbt{k}.cons_map)','LineColor','none');
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
                    text(rbt{j}.traj(1,end)-3, rbt{j}.traj(2,end)-3, sprintf('%d',j),'FontSize',10);
                    
                    % draw target trajectory
                    line_hdl = line(fld.target.traj(1,:), fld.target.traj(2,:));
                    set(line_hdl,'Marker','.','Color','k','MarkerSize',3,'LineWidth',2);
                    plot(fld.target.pos(1), fld.target.pos(2), 'k+','MarkerSize',25,'LineWidth',3);
                    plot(fld.target.traj(1,:), fld.target.traj(2,:),'LineWidth',3);
                    set(gca,'fontsize',30)
                end
                title(sprintf('Consensus Robot %d',k))
                xlabel(['Step=',num2str(count)],'FontSize',30);
                tmp_fig_cnt = tmp_fig_cnt+1;
            end
            %}
            
            %% Centralized
            %{
            % plot figures for central map
            
            tmp_hd = figure (tmp_fig_cnt); % handle for plot of a single robot's target PDF
            clf(tmp_hd);
            shading interp
            contourf((rbt{1}.cent_map)','LineColor','none');
            load('MyColorMap','mymap')
            colormap(mymap);
            colorbar
            xlabel(['Step=',num2str(count)],'FontSize',30);
            
            hold on;
            
            % draw robot trajectory
            for j=1:this.num_robot
                line_hdl = line(rbt{j}.traj(1,:), rbt{j}.traj(2,:));
                set(line_hdl,'Marker','.','Color','g','MarkerSize',3,'LineWidth',2);
                plot(rbt{j}.traj(1,end), rbt{j}.traj(2,end), 'p','Color','g','MarkerSize',25,'LineWidth',1.5);
                text(rbt{j}.traj(1,end)-3, rbt{j}.traj(2,end)-3, sprintf('%d',j),'FontSize',10);
            end
            
            % draw target trajectory
            line_hdl = line(fld.target.traj(1,:), fld.target.traj(2,:));
            set(line_hdl,'Marker','.','Color','k','MarkerSize',3,'LineWidth',2);
            plot(fld.target.pos(1), fld.target.pos(2), 'k+','MarkerSize',25,'LineWidth',3);
            plot(fld.target.traj(1,:), fld.target.traj(2,:), 'LineWidth',3);
            set(gca,'fontsize',30)
            title('CF Robot 1')
%             tmp_fig_cnt = tmp_fig_cnt+1;
            %}
        end
        
        function this = compareMetrics(this)
            tmp_sim_res = this.sim_res;
            for jj = 1:this.trial_num
                for ii = 1:this.num_robot
%                     display(ii)
                    % ml error
                    tmp_sim_res.ml_err_dbf(ii,jj,:) = this.rbt_set{jj}.rbt{ii}.ml_err_dbf;
                    tmp_sim_res.ml_err_cons(ii,jj,:) = this.rbt_set{jj}.rbt{ii}.ml_err_cons;
                    
                    % norm of cov of pdf
                    tmp_sim_res.pdf_norm_dbf(ii,jj,:) = this.rbt_set{jj}.rbt{ii}.pdf_norm_dbf;
                    tmp_sim_res.pdf_norm_cons(ii,jj,:) = this.rbt_set{jj}.rbt{ii}.pdf_norm_cons;
                    
                    % entropy of pdf
                    tmp_sim_res.ent_dbf(ii,jj,:) = this.rbt_set{jj}.rbt{ii}.ent_dbf;
                    tmp_sim_res.ent_cons(ii,jj,:) = this.rbt_set{jj}.rbt{ii}.ent_cons;
                end
                tmp_sim_res.ml_err_cent(jj,:) = this.rbt_set{jj}.rbt{1}.ml_err_cent;
                tmp_sim_res.pdf_norm_cent(jj,:) = this.rbt_set{jj}.rbt{1}.pdf_norm_cent;
                tmp_sim_res.ent_cent(jj,:) = this.rbt_set{jj}.rbt{1}.ent_cent;
            end
            
            for ii = 1:this.num_robot
                % ml error
                % dbf
                tmp_ml_err_dbf = squeeze(tmp_sim_res.ml_err_dbf(ii,:,:));
                tmp_sim_res.ml_err_dbf_mean(ii,:) = mean(tmp_ml_err_dbf,1);
                tmp_sim_res.ml_err_dbf_cov(ii,:) = diag(cov(tmp_ml_err_dbf))';
                
                % consensus
                tmp_ml_err_cons = squeeze(tmp_sim_res.ml_err_cons(ii,:,:));
                tmp_sim_res.ml_err_cons_mean(ii,:) = mean(tmp_ml_err_cons,1);
                tmp_sim_res.ml_err_cons_cov(ii,:) = diag(cov(tmp_ml_err_cons)');
                
                % norm of cov of pdf
                % dbf
                tmp_pdf_norm_dbf = squeeze(tmp_sim_res.pdf_norm_dbf(ii,:,:));
                tmp_sim_res.pdf_norm_dbf_mean(ii,:) = mean(tmp_pdf_norm_dbf,1);
                tmp_sim_res.pdf_norm_dbf_cov(ii,:) = diag(cov(tmp_pdf_norm_dbf)');
                
                % consensus
                tmp_pdf_norm_cons = squeeze(tmp_sim_res.pdf_norm_cons(ii,:,:));
                tmp_sim_res.pdf_norm_cons_mean(ii,:) = mean(tmp_pdf_norm_cons,1);
                tmp_sim_res.pdf_norm_cons_cov(ii,:) = diag(cov(tmp_pdf_norm_cons)');
                
                % entropy of pdf
                % dbf
                tmp_ent_dbf = squeeze(tmp_sim_res.ent_dbf(ii,:,:));
                tmp_sim_res.ent_dbf_mean(ii,:) = mean(tmp_ent_dbf,1);
                tmp_sim_res.ent_dbf_cov(ii,:) = diag(cov(tmp_ent_dbf)');
                
                % consensus
                tmp_ent_cons = squeeze(tmp_sim_res.ent_cons(ii,:,:));
                tmp_sim_res.ent_cons_mean(ii,:) = mean(tmp_ent_cons,1);
                tmp_sim_res.ent_cons_cov(ii,:) = diag(cov(tmp_ent_cons)');
            end
            
            % ml error
            % centralized
            tmp_ml_err_cent = tmp_sim_res.ml_err_cent;
            tmp_sim_res.ml_err_cent_mean = mean(tmp_ml_err_cent,1);
            tmp_sim_res.ml_err_cent_cov = diag(cov(tmp_ml_err_cent)');
            
            % norm of cov of pdf
            % centralized
            tmp_pdf_norm_cent = tmp_sim_res.pdf_norm_cent;
            tmp_sim_res.pdf_norm_cent_mean = mean(tmp_pdf_norm_cent,1);
            tmp_sim_res.pdf_norm_cent_cov = diag(cov(tmp_pdf_norm_cent)');
            
            % entropy of pdf
            % centralized
            tmp_ent_cent = tmp_sim_res.ent_cent;
            tmp_sim_res.ent_cent_mean = mean(tmp_ent_cent,1);
            tmp_sim_res.ent_cent_cov = diag(cov(tmp_ent_cent)');
            
            %% %%%%%%%%%%%%%% plot the performance metrics %%%%%%%%%%%%%%%%%
            plot_rbt_idx = this.sim_r_idx; % draw robot 1, 3, 5
            tmp_fig_cnt = this.fig_cnt+7;
            % ml error
%             tmp_fig_cnt = tmp_fig_cnt+1;
            hf_err = figure(tmp_fig_cnt);
            line_clr = ['r','g','b','c','m','k'];
            line_marker = {'o','*','s','d','^','h'};
            count = this.sim_len;
            % for LIFO-DBF, we draw different robot's performance metrics
            for ii = [1,3,5]%plot_rbt_idx
                plot(1:count-2,tmp_sim_res.ml_err_dbf_mean(ii,1:count-2),line_clr(ii),'LineWidth',2,'Marker',line_marker{ii},'MarkerSize',2); hold on;
%                 semilogy(1:count-2,tmp_sim_res.ml_err_dbf_mean(ii,1:count-2),line_clr(ii),'LineWidth',2,'Marker',line_marker{ii},'MarkerSize',2); hold on;
%                 errorbar(1:count-2,tmp_sim_res.ml_err_dbf_mean(ii,1:count-2),sqrt(tmp_sim_res.ml_err_dbf_cov(ii,1:count-2)),...
%                         line_clr(ii),'LineWidth',2,'Marker',line_marker{ii},'MarkerSize',2); hold on;
            end
            
            % for consensus, we draw one robot's performance metrics.
            plot(1:count-2,tmp_sim_res.ml_err_cons_mean(1,1:count-2),line_clr(2),'LineStyle','--','LineWidth',2,'Marker',line_marker{2},'MarkerSize',2); hold on;
%             semilogy(1:count-2,tmp_sim_res.ml_err_cons_mean(1,1:count-2),line_clr(2),'LineStyle','--','LineWidth',2,'Marker',line_marker{2},'MarkerSize',2); hold on;
            % errorbar(1:count-2,tmp_sim_res.ml_err_cons_mean(1,1:count-2),sqrt(tmp_sim_res.ml_err_cons_cov(1,1:count-2)),...
            %     line_clr(1),'LineStyle','--','LineWidth',2,'Marker',line_marker{1},'MarkerSize',2); hold on;
            
            % only on centralized filter
            plot(1:count-2,tmp_sim_res.ml_err_cent_mean(1:count-2),line_clr(6),'LineStyle','-.','LineWidth',2,'Marker',line_marker{6},'MarkerSize',2); hold on;
%             semilogy(1:count-2,tmp_sim_res.ml_err_cent_mean(1:count-2),line_clr(6),'LineStyle','-.','LineWidth',2,'Marker',line_marker{6},'MarkerSize',2); hold on;
            % errorbar(1:count-2,tmp_sim_res.ml_err_cent_mean(1:count-2),sqrt(tmp_sim_res.ml_err_cent_cov(1:count-2)),...
            %     line_clr(6),'LineStyle','-.','LineWidth',2,'Marker',line_marker{6},'MarkerSize',2); hold on;
            xlim([0,count-1])
            
            % add legend
            [~, hobj1] = legend('DBF-R1','DBF-R3','DBF-R5','Consen','Central');
            textobj = findobj(hobj1, 'type', 'text');
            set(textobj, 'fontsize', 15);
            
            title('Target Position Error','FontSize',30);
            set(gca,'fontsize',30)
            xlabel('Time (Step)','FontSize',30);
            ylabel('Position Error','FontSize',30);
            
            % entropy
            tmp_fig_cnt = tmp_fig_cnt+1;
            hf_ent = figure(tmp_fig_cnt);
            line_clr = ['r','g','b','c','m','k'];
            line_marker = {'o','*','s','d','^','h'};
            for ii=[1,3,5]%plot_rbt_idx
                plot(1:count-2,tmp_sim_res.ent_dbf_mean(ii,1:count-2),line_clr(ii),'LineWidth',2,'Marker',line_marker{ii},'MarkerSize',2); hold on;
                %     errorbar(1:count-2,tmp_sim_res.entropy_dbf_mean(i,1:count-2),sqrt(tmp_sim_res.entropy_dbf_cov(i,1:count-2)),line_clr(i),'LineWidth',2,'Marker',line_marker{i},'MarkerSize',2); hold on;
            end
            
            plot(1:count-2,tmp_sim_res.ent_cons_mean(1,1:count-2),line_clr(2),'LineStyle','--','LineWidth',2,'Marker',line_marker{2},'MarkerSize',2); hold on;
            % errorbar(1:count-2,tmp_sim_res.entropy_cons_mean(1,1:count-2),sqrt(tmp_sim_res.entropy_cons_cov(1,1:count-2)),line_clr(1),'LineStyle','--','LineWidth',2,'Marker',line_marker{i},'MarkerSize',2); hold on;
            
            plot(1:count-2,tmp_sim_res.ent_cent_mean(1:count-2),line_clr(6),'LineStyle','-.','LineWidth',2,'Marker',line_marker{6},'MarkerSize',2); hold on;
            % errorbar(1:count-2,tmp_sim_res.entropy_cent_mean(1:count-2),sqrt(tmp_sim_res.entropy_cent_cov(1:count-2)),line_clr(6),'LineStyle','-.','LineWidth',2,'Marker',line_marker{i},'MarkerSize',2); hold on;
            xlim([0,count-1])
            
            % add legend
            [~, hobj3] = legend('DBF-R1','DBF-R3','DBF-R5','Consen','Central');
            textobj = findobj(hobj3, 'type', 'text');
            set(textobj, 'fontsize', 15);
            
            title('Entropy of the Target PDF','FontSize',30);
            set(gca,'fontsize',30)
            xlabel('Time (Step)','FontSize',30);
            ylabel('Entropy','FontSize',30);
            
            this.sim_res = tmp_sim_res;
        end
        
        function this = compareMetricsExp(this)
            % used for analyzing experiment data
            tmp_sim_res.ml_err_dbf = zeros(this.num_robot,this.sim_len);
            tmp_sim_res.ent_dbf = zeros(this.num_robot,this.sim_len);
            tmp_sim_res.ml_err_cons = zeros(this.num_robot,this.sim_len);
            tmp_sim_res.ent_cons = zeros(this.num_robot,this.sim_len);
            tmp_sim_res.ml_err_cent = zeros(1,this.sim_len);
            tmp_sim_res.ent_cent = zeros(1,this.sim_len);
%             tmp_sim_res = this.sim_res;
                for ii = 1:this.num_robot
                    %                     display(ii)
                    % ml error
                    tmp_sim_res.ml_err_dbf(ii,:) = this.rbt_set{1}.rbt{ii}.ml_err_dbf;
                    tmp_sim_res.ml_err_cons(ii,:) = this.rbt_set{1}.rbt{ii}.ml_err_cons;
                    
                    % norm of cov of pdf
%                     tmp_sim_res.pdf_norm_dbf(ii,jj,:) = this.rbt_set{jj}.rbt{ii}.pdf_norm_dbf;
%                     tmp_sim_res.pdf_norm_cons(ii,jj,:) = this.rbt_set{jj}.rbt{ii}.pdf_norm_cons;
                    
                    % entropy of pdf
                    tmp_sim_res.ent_dbf(ii,:) = this.rbt_set{1}.rbt{ii}.ent_dbf;
                    tmp_sim_res.ent_cons(ii,:) = this.rbt_set{1}.rbt{ii}.ent_cons;
                end
                tmp_sim_res.ml_err_cent(:) = this.rbt_set{1}.rbt{1}.ml_err_cent;
%                 tmp_sim_res.pdf_norm_cent(jj,:) = this.rbt_set{jj}.rbt{1}.pdf_norm_cent;
                tmp_sim_res.ent_cent(:) = this.rbt_set{1}.rbt{1}.ent_cent;
            
%             
%             for ii = 1:this.num_robot
%                 % ml error
%                 % dbf
%                 tmp_ml_err_dbf = squeeze(tmp_sim_res.ml_err_dbf(ii,:,:));
%                 tmp_sim_res.ml_err_dbf_mean(ii,:) = mean(tmp_ml_err_dbf,1);
%                 tmp_sim_res.ml_err_dbf_cov(ii,:) = diag(cov(tmp_ml_err_dbf))';
%                 
% %                 % consensus
% %                 tmp_ml_err_cons = squeeze(tmp_sim_res.ml_err_cons(ii,:,:));
% %                 tmp_sim_res.ml_err_cons_mean(ii,:) = mean(tmp_ml_err_cons,1);
% %                 tmp_sim_res.ml_err_cons_cov(ii,:) = diag(cov(tmp_ml_err_cons)');
%                 
%                 % norm of cov of pdf
%                 % dbf
%                 tmp_pdf_norm_dbf = squeeze(tmp_sim_res.pdf_norm_dbf(ii,:,:));
%                 tmp_sim_res.pdf_norm_dbf_mean(ii,:) = mean(tmp_pdf_norm_dbf,1);
%                 tmp_sim_res.pdf_norm_dbf_cov(ii,:) = diag(cov(tmp_pdf_norm_dbf)');
%                 
% %                 % consensus
% %                 tmp_pdf_norm_cons = squeeze(tmp_sim_res.pdf_norm_cons(ii,:,:));
% %                 tmp_sim_res.pdf_norm_cons_mean(ii,:) = mean(tmp_pdf_norm_cons,1);
% %                 tmp_sim_res.pdf_norm_cons_cov(ii,:) = diag(cov(tmp_pdf_norm_cons)');
%                 
%                 % entropy of pdf
%                 % dbf
%                 tmp_ent_dbf = squeeze(tmp_sim_res.ent_dbf(ii,:,:));
%                 tmp_sim_res.ent_dbf_mean(ii,:) = mean(tmp_ent_dbf,1);
%                 tmp_sim_res.ent_dbf_cov(ii,:) = diag(cov(tmp_ent_dbf)');
%                 
% %                 % consensus
% %                 tmp_ent_cons = squeeze(tmp_sim_res.ent_cons(ii,:,:));
% %                 tmp_sim_res.ent_cons_mean(ii,:) = mean(tmp_ent_cons,1);
% %                 tmp_sim_res.ent_cons_cov(ii,:) = diag(cov(tmp_ent_cons)');
%             end
%             
            % ml error
            % centralized
%             tmp_ml_err_cent = tmp_sim_res.ml_err_cent;
%             tmp_sim_res.ml_err_cent_mean = mean(tmp_ml_err_cent,1);
%             tmp_sim_res.ml_err_cent_cov = diag(cov(tmp_ml_err_cent)');
            
            % norm of cov of pdf
            % centralized
%             tmp_pdf_norm_cent = tmp_sim_res.pdf_norm_cent;
%             tmp_sim_res.pdf_norm_cent_mean = mean(tmp_pdf_norm_cent,1);
%             tmp_sim_res.pdf_norm_cent_cov = diag(cov(tmp_pdf_norm_cent)');
            
            % entropy of pdf
            % centralized
%             tmp_ent_cent = tmp_sim_res.ent_cent;
%             tmp_sim_res.ent_cent_mean = mean(tmp_ent_cent,1);
%             tmp_sim_res.ent_cent_cov = diag(cov(tmp_ent_cent)');
            
            %% %%%%%%%%%%%%%% plot the performance metrics %%%%%%%%%%%%%%%%%
            plot_rbt_idx = this.sim_r_idx; % draw robot 1, 3, 5
            tmp_fig_cnt = this.fig_cnt+7;
            % ml error
            %             tmp_fig_cnt = tmp_fig_cnt+1;
            hf_err = figure(tmp_fig_cnt);
            line_clr = ['r','g','b','c','m','k'];
            line_marker = {'o','*','s','d','^','h'};
            count = this.sim_len;
            % for LIFO-DBF, we draw different robot's performance metrics
            for ii = plot_rbt_idx
                plot(1:count-2,tmp_sim_res.ml_err_dbf(ii,1:count-2),line_clr(ii),'LineWidth',2,'Marker',line_marker{ii},'MarkerSize',2); hold on;
                %                 semilogy(1:count-2,tmp_sim_res.ml_err_dbf_mean(ii,1:count-2),line_clr(ii),'LineWidth',2,'Marker',line_marker{ii},'MarkerSize',2); hold on;
                %                 errorbar(1:count-2,tmp_sim_res.ml_err_dbf_mean(ii,1:count-2),sqrt(tmp_sim_res.ml_err_dbf_cov(ii,1:count-2)),...
                %                         line_clr(ii),'LineWidth',2,'Marker',line_marker{ii},'MarkerSize',2); hold on;
            end
            
            % for consensus, we draw one robot's performance metrics.
            plot(1:count-2,tmp_sim_res.ml_err_cons(1,1:count-2),line_clr(2),'LineStyle','--','LineWidth',2,'Marker',line_marker{2},'MarkerSize',2); hold on;
            %             semilogy(1:count-2,tmp_sim_res.ml_err_cons_mean(1,1:count-2),line_clr(2),'LineStyle','--','LineWidth',2,'Marker',line_marker{2},'MarkerSize',2); hold on;
            % errorbar(1:count-2,tmp_sim_res.ml_err_cons_mean(1,1:count-2),sqrt(tmp_sim_res.ml_err_cons_cov(1,1:count-2)),...
            %     line_clr(1),'LineStyle','--','LineWidth',2,'Marker',line_marker{1},'MarkerSize',2); hold on;
            
            % only on centralized filter
            plot(1:count-2,tmp_sim_res.ml_err_cent(1:count-2),line_clr(6),'LineStyle','-.','LineWidth',2,'Marker',line_marker{6},'MarkerSize',2); hold on;
            %             semilogy(1:count-2,tmp_sim_res.ml_err_cent_mean(1:count-2),line_clr(6),'LineStyle','-.','LineWidth',2,'Marker',line_marker{6},'MarkerSize',2); hold on;
            % errorbar(1:count-2,tmp_sim_res.ml_err_cent_mean(1:count-2),sqrt(tmp_sim_res.ml_err_cent_cov(1:count-2)),...
            %     line_clr(6),'LineStyle','-.','LineWidth',2,'Marker',line_marker{6},'MarkerSize',2); hold on;
            xlim([0,count-1])
            
            % add legend
            [~, hobj1] = legend('DBF-R1','DBF-R3','DBF-R5');%,'Consen','Central');
            textobj = findobj(hobj1, 'type', 'text');
            set(textobj, 'fontsize', 15);
            
            title('Target Position Error','FontSize',30);
            set(gca,'fontsize',30)
            xlabel('Time (Step)','FontSize',30);
            ylabel('Position Error','FontSize',30);
            
            % entropy
            tmp_fig_cnt = tmp_fig_cnt+1;
            hf_ent = figure(tmp_fig_cnt);
            line_clr = ['r','g','b','c','m','k'];
            line_marker = {'o','*','s','d','^','h'};
            for ii = plot_rbt_idx
                plot(1:count-2,tmp_sim_res.ent_dbf(ii,1:count-2),line_clr(ii),'LineWidth',2,'Marker',line_marker{ii},'MarkerSize',2); hold on;
                %     errorbar(1:count-2,tmp_sim_res.entropy_dbf_mean(i,1:count-2),sqrt(tmp_sim_res.entropy_dbf_cov(i,1:count-2)),line_clr(i),'LineWidth',2,'Marker',line_marker{i},'MarkerSize',2); hold on;
            end
            
            plot(1:count-2,tmp_sim_res.ent_cons(1,1:count-2),line_clr(2),'LineStyle','--','LineWidth',2,'Marker',line_marker{2},'MarkerSize',2); hold on;
            % errorbar(1:count-2,tmp_sim_res.entropy_cons_mean(1,1:count-2),sqrt(tmp_sim_res.entropy_cons_cov(1,1:count-2)),line_clr(1),'LineStyle','--','LineWidth',2,'Marker',line_marker{i},'MarkerSize',2); hold on;
            
            plot(1:count-2,tmp_sim_res.ent_cent(1:count-2),line_clr(6),'LineStyle','-.','LineWidth',2,'Marker',line_marker{6},'MarkerSize',2); hold on;
            % errorbar(1:count-2,tmp_sim_res.entropy_cent_mean(1:count-2),sqrt(tmp_sim_res.entropy_cent_cov(1:count-2)),line_clr(6),'LineStyle','-.','LineWidth',2,'Marker',line_marker{i},'MarkerSize',2); hold on;
            xlim([0,count-1])
            
            % add legend
            [~, hobj3] = legend('DBF-R1','DBF-R3','DBF-R5');%,'Consen','Central');
            textobj = findobj(hobj3, 'type', 'text');
            set(textobj, 'fontsize', 15);
            
            title('Entropy of the Target PDF','FontSize',30);
            set(gca,'fontsize',30)
            xlabel('Time (Step)','FontSize',30);
            ylabel('Entropy','FontSize',30);
            
            this.sim_res = tmp_sim_res;
        end
        
        % remove this function is the above one works
        %{
        function this = compareMetricsExp(this)
            % draw metrics for LIFO-DBF only. used for analyzing experiment
            % data
            tmp_sim_res.ml_err_dbf = zeros(this.num_robot,this.sim_len);
            tmp_sim_res.ent_dbf = zeros(this.num_robot,this.sim_len);
%             tmp_sim_res = this.sim_res;
                for ii = 1:this.num_robot
                    %                     display(ii)
                    % ml error
                    tmp_sim_res.ml_err_dbf(ii,:) = this.rbt_set{1}.rbt{ii}.ml_err_dbf;
%                     tmp_sim_res.ml_err_cons(ii,jj,:) = this.rbt_set{jj}.rbt{ii}.ml_err_cons;
                    
                    % norm of cov of pdf
%                     tmp_sim_res.pdf_norm_dbf(ii,jj,:) = this.rbt_set{jj}.rbt{ii}.pdf_norm_dbf;
%                     tmp_sim_res.pdf_norm_cons(ii,jj,:) = this.rbt_set{jj}.rbt{ii}.pdf_norm_cons;
                    
                    % entropy of pdf
                    tmp_sim_res.ent_dbf(ii,:) = this.rbt_set{1}.rbt{ii}.ent_dbf;
%                     tmp_sim_res.ent_cons(ii,jj,:) = this.rbt_set{jj}.rbt{ii}.ent_cons;
                end
%                 tmp_sim_res.ml_err_cent(jj,:) = this.rbt_set{jj}.rbt{1}.ml_err_cent;
%                 tmp_sim_res.pdf_norm_cent(jj,:) = this.rbt_set{jj}.rbt{1}.pdf_norm_cent;
%                 tmp_sim_res.ent_cent(jj,:) = this.rbt_set{jj}.rbt{1}.ent_cent;
            
%             
%             for ii = 1:this.num_robot
%                 % ml error
%                 % dbf
%                 tmp_ml_err_dbf = squeeze(tmp_sim_res.ml_err_dbf(ii,:,:));
%                 tmp_sim_res.ml_err_dbf_mean(ii,:) = mean(tmp_ml_err_dbf,1);
%                 tmp_sim_res.ml_err_dbf_cov(ii,:) = diag(cov(tmp_ml_err_dbf))';
%                 
% %                 % consensus
% %                 tmp_ml_err_cons = squeeze(tmp_sim_res.ml_err_cons(ii,:,:));
% %                 tmp_sim_res.ml_err_cons_mean(ii,:) = mean(tmp_ml_err_cons,1);
% %                 tmp_sim_res.ml_err_cons_cov(ii,:) = diag(cov(tmp_ml_err_cons)');
%                 
%                 % norm of cov of pdf
%                 % dbf
%                 tmp_pdf_norm_dbf = squeeze(tmp_sim_res.pdf_norm_dbf(ii,:,:));
%                 tmp_sim_res.pdf_norm_dbf_mean(ii,:) = mean(tmp_pdf_norm_dbf,1);
%                 tmp_sim_res.pdf_norm_dbf_cov(ii,:) = diag(cov(tmp_pdf_norm_dbf)');
%                 
% %                 % consensus
% %                 tmp_pdf_norm_cons = squeeze(tmp_sim_res.pdf_norm_cons(ii,:,:));
% %                 tmp_sim_res.pdf_norm_cons_mean(ii,:) = mean(tmp_pdf_norm_cons,1);
% %                 tmp_sim_res.pdf_norm_cons_cov(ii,:) = diag(cov(tmp_pdf_norm_cons)');
%                 
%                 % entropy of pdf
%                 % dbf
%                 tmp_ent_dbf = squeeze(tmp_sim_res.ent_dbf(ii,:,:));
%                 tmp_sim_res.ent_dbf_mean(ii,:) = mean(tmp_ent_dbf,1);
%                 tmp_sim_res.ent_dbf_cov(ii,:) = diag(cov(tmp_ent_dbf)');
%                 
% %                 % consensus
% %                 tmp_ent_cons = squeeze(tmp_sim_res.ent_cons(ii,:,:));
% %                 tmp_sim_res.ent_cons_mean(ii,:) = mean(tmp_ent_cons,1);
% %                 tmp_sim_res.ent_cons_cov(ii,:) = diag(cov(tmp_ent_cons)');
%             end
%             
            % ml error
            % centralized
%             tmp_ml_err_cent = tmp_sim_res.ml_err_cent;
%             tmp_sim_res.ml_err_cent_mean = mean(tmp_ml_err_cent,1);
%             tmp_sim_res.ml_err_cent_cov = diag(cov(tmp_ml_err_cent)');
            
            % norm of cov of pdf
            % centralized
%             tmp_pdf_norm_cent = tmp_sim_res.pdf_norm_cent;
%             tmp_sim_res.pdf_norm_cent_mean = mean(tmp_pdf_norm_cent,1);
%             tmp_sim_res.pdf_norm_cent_cov = diag(cov(tmp_pdf_norm_cent)');
            
            % entropy of pdf
            % centralized
%             tmp_ent_cent = tmp_sim_res.ent_cent;
%             tmp_sim_res.ent_cent_mean = mean(tmp_ent_cent,1);
%             tmp_sim_res.ent_cent_cov = diag(cov(tmp_ent_cent)');
            
            %% %%%%%%%%%%%%%% plot the performance metrics %%%%%%%%%%%%%%%%%
            plot_rbt_idx = this.sim_r_idx; % draw robot 1, 3, 5
            tmp_fig_cnt = this.fig_cnt+7;
            % ml error
            %             tmp_fig_cnt = tmp_fig_cnt+1;
            hf_err = figure(tmp_fig_cnt);
            line_clr = ['r','g','b','c','m','k'];
            line_marker = {'o','*','s','d','^','h'};
            count = this.sim_len;
            % for LIFO-DBF, we draw different robot's performance metrics
            for ii = plot_rbt_idx
                plot(1:count-2,tmp_sim_res.ml_err_dbf(ii,1:count-2),line_clr(ii),'LineWidth',2,'Marker',line_marker{ii},'MarkerSize',2); hold on;
                %                 semilogy(1:count-2,tmp_sim_res.ml_err_dbf_mean(ii,1:count-2),line_clr(ii),'LineWidth',2,'Marker',line_marker{ii},'MarkerSize',2); hold on;
                %                 errorbar(1:count-2,tmp_sim_res.ml_err_dbf_mean(ii,1:count-2),sqrt(tmp_sim_res.ml_err_dbf_cov(ii,1:count-2)),...
                %                         line_clr(ii),'LineWidth',2,'Marker',line_marker{ii},'MarkerSize',2); hold on;
            end
            
            % for consensus, we draw one robot's performance metrics.
%             plot(1:count-2,tmp_sim_res.ml_err_cons_mean(1,1:count-2),line_clr(2),'LineStyle','--','LineWidth',2,'Marker',line_marker{2},'MarkerSize',2); hold on;
            %             semilogy(1:count-2,tmp_sim_res.ml_err_cons_mean(1,1:count-2),line_clr(2),'LineStyle','--','LineWidth',2,'Marker',line_marker{2},'MarkerSize',2); hold on;
            % errorbar(1:count-2,tmp_sim_res.ml_err_cons_mean(1,1:count-2),sqrt(tmp_sim_res.ml_err_cons_cov(1,1:count-2)),...
            %     line_clr(1),'LineStyle','--','LineWidth',2,'Marker',line_marker{1},'MarkerSize',2); hold on;
            
            % only on centralized filter
%             plot(1:count-2,tmp_sim_res.ml_err_cent_mean(1:count-2),line_clr(6),'LineStyle','-.','LineWidth',2,'Marker',line_marker{6},'MarkerSize',2); hold on;
            %             semilogy(1:count-2,tmp_sim_res.ml_err_cent_mean(1:count-2),line_clr(6),'LineStyle','-.','LineWidth',2,'Marker',line_marker{6},'MarkerSize',2); hold on;
            % errorbar(1:count-2,tmp_sim_res.ml_err_cent_mean(1:count-2),sqrt(tmp_sim_res.ml_err_cent_cov(1:count-2)),...
            %     line_clr(6),'LineStyle','-.','LineWidth',2,'Marker',line_marker{6},'MarkerSize',2); hold on;
            xlim([0,count-1])
            
            % add legend
            [~, hobj1] = legend('DBF-R1','DBF-R3','DBF-R5');%,'Consen','Central');
            textobj = findobj(hobj1, 'type', 'text');
            set(textobj, 'fontsize', 15);
            
            title('Target Position Error','FontSize',30);
            set(gca,'fontsize',30)
            xlabel('Time (Step)','FontSize',30);
            ylabel('Position Error','FontSize',30);
            
            % entropy
            tmp_fig_cnt = tmp_fig_cnt+1;
            hf_ent = figure(tmp_fig_cnt);
            line_clr = ['r','g','b','c','m','k'];
            line_marker = {'o','*','s','d','^','h'};
            for ii = plot_rbt_idx
                plot(1:count-2,tmp_sim_res.ent_dbf(ii,1:count-2),line_clr(ii),'LineWidth',2,'Marker',line_marker{ii},'MarkerSize',2); hold on;
                %     errorbar(1:count-2,tmp_sim_res.entropy_dbf_mean(i,1:count-2),sqrt(tmp_sim_res.entropy_dbf_cov(i,1:count-2)),line_clr(i),'LineWidth',2,'Marker',line_marker{i},'MarkerSize',2); hold on;
            end
            
%             plot(1:count-2,tmp_sim_res.ent_cons_mean(1,1:count-2),line_clr(2),'LineStyle','--','LineWidth',2,'Marker',line_marker{2},'MarkerSize',2); hold on;
            % errorbar(1:count-2,tmp_sim_res.entropy_cons_mean(1,1:count-2),sqrt(tmp_sim_res.entropy_cons_cov(1,1:count-2)),line_clr(1),'LineStyle','--','LineWidth',2,'Marker',line_marker{i},'MarkerSize',2); hold on;
            
%             plot(1:count-2,tmp_sim_res.ent_cent_mean(1:count-2),line_clr(6),'LineStyle','-.','LineWidth',2,'Marker',line_marker{6},'MarkerSize',2); hold on;
            % errorbar(1:count-2,tmp_sim_res.entropy_cent_mean(1:count-2),sqrt(tmp_sim_res.entropy_cent_cov(1:count-2)),line_clr(6),'LineStyle','-.','LineWidth',2,'Marker',line_marker{i},'MarkerSize',2); hold on;
            xlim([0,count-1])
            
            % add legend
            [~, hobj3] = legend('DBF-R1','DBF-R3','DBF-R5');%,'Consen','Central');
            textobj = findobj(hobj3, 'type', 'text');
            set(textobj, 'fontsize', 15);
            
            title('Entropy of the Target PDF','FontSize',30);
            set(gca,'fontsize',30)
            xlabel('Time (Step)','FontSize',30);
            ylabel('Entropy','FontSize',30);
            
            this.sim_res = tmp_sim_res;
        end
        %}
        
        function file_name = saveSimData(this)
            % make the file name for simulation data
            switch this.selection
                case 1,  tag = 'sta_sen_sta_tar';
                case 2,  tag = 'sta_sen_mov_tar';
                case 3,  tag = 'mov_sen_sta_tar';
                case 4,  tag = 'mov_sen_mov_tar';
            end
            
            switch this.sensor_set_type
                case 'brg', tag2 = 'brg';
                case 'ran', tag2 = 'ran';
                case 'rb', tag2 = 'rb';
                case 'htr', tag2 = 'hetero';
            end
            
            file_name = sprintf('./figures/Journal/metrics_plot/metrics_%s_%s_%s.mat',tag2,tag,datestr(now,1));            
        end
        
        function file_name = saveExpData(this)
            % make the file name for simulation data
            switch this.selection
                case 1,  tag = 'sta_sen_sta_tar';
                case 2,  tag = 'sta_sen_mov_tar';
                case 3,  tag = 'mov_sen_sta_tar';
                case 4,  tag = 'mov_sen_mov_tar';
            end
            
            tag2 = 'sonar';
            
            file_name = sprintf('./figures/Journal/metrics_plot/metrics_%s_%s_%s.mat',tag2,tag,datestr(now,1));            
        end        
    end
end