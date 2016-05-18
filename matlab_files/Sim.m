classdef Sim
    properties
        dt; % discretization time interval
    end
    
    methods
        
        function obj = Sim(inPara)
           obj.dt = inPara.dt; 
        end
        
        function computeMetrics(sim)
         %% %%%%%%%%%%%% Computing Performance Metrics %%%%%%%%%%%%%%%%%%%
         
         % ML error
         for i = 1:NumOfRobot
             % DBF
             [tmp_x1,tmp_y1] = find(rbt(i).map == max(rbt(i).map(:)));
             if length(tmp_x1) > 1
                 tmp_idx = randi(length(tmp_x1),1,1);
             else
                 tmp_idx = 1;
             end
             rbt(i).ml_dbf(:,count) = [tmp_x1(tmp_idx);tmp_y1(tmp_idx)];
             rbt(i).ml_err_dbf(count) = norm(rbt(i).ml_dbf(:,count)-[fld.tx;fld.ty]);
             
             % concensus
             [tmp_x2,tmp_y2] = find(rbt_cons(i).map == max(rbt_cons(i).map(:)));
             if length(tmp_x2) > 1
                 tmp_idx2 = randi(length(tmp_x2),1,1);
             else
                 tmp_idx2 = 1;
             end
             rbt_cons(i).ml_cons(:,count) = [tmp_x2(tmp_idx2);tmp_y2(tmp_idx2)];
             rbt_cons(i).ml_err_cons(count) = norm(rbt_cons(i).ml_cons(:,count)-[fld.tx;fld.ty]);
         end
         
         % centralized
         [tmp_x3,tmp_y3] = find(rbt_cent.map == max(rbt_cent.map(:)));
         if length(tmp_x3) > 1
             tmp_idx3 = randi(length(tmp_x3),1,1);
         else
             tmp_idx3 = 1;
         end
         rbt_cent.ml_cent(:,count) = [tmp_x3(tmp_idx3);tmp_y3(tmp_idx3)];
         rbt_cent.ml_err_cent(count) = norm(rbt_cent.ml_cent(:,count)-[fld.tx;fld.ty]);
         
         % Covariance of posterior pdf
         for i=1:NumOfRobot
             % DBF
             tmp_map1 = rbt(i).map;
             % this avoids the error when some grid has zeros probability
             tmp_map1(tmp_map1 <= realmin) = realmin;
             
             % compute covariance of distribution
             dif1 = pt' - [(1+fld.x)/2;(1+fld.y)/2]*ones(1,size(pt',2));
             cov_p1 = zeros(2,2);
             for jj = 1:size(pt',2)
                 cov_p1 = cov_p1 + dif1(:,jj)*dif1(:,jj)'*tmp_map1(pt(jj,1),pt(jj,2));
             end
             rbt(i).pdf_cov{count} = cov_p1;
             rbt(i).pdf_norm_dbf(count) = norm(cov_p1,'fro');
             
             % concensus
             tmp_map2 = rbt_cons(i).map;
             % this avoids the error when some grid has zeros probability
             tmp_map2(tmp_map2 <= realmin) = realmin;
             
             % compute covariance of distribution
             dif2 = pt' - [(1+fld.x)/2;(1+fld.y)/2]*ones(1,size(pt',2));
             cov_p2 = zeros(2,2);
             for jj = 1:size(pt',2)
                 cov_p2 = cov_p2 + dif2(:,jj)*dif2(:,jj)'*tmp_map2(pt(jj,1),pt(jj,2));
             end
             rbt_cons(i).pdf_cov_cons{count} = cov_p2;
             rbt_cons(i).pdf_norm_cons(count) = norm(cov_p2,'fro');
         end
         
         % centralized
         tmp_map3 = rbt_cent.map;
         % this avoids the error when some grid has zeros probability
         tmp_map3(tmp_map3 <= realmin) = realmin;
         
         % compute covariance of distribution
         dif3 = pt' - [(1+fld.x)/2;(1+fld.y)/2]*ones(1,size(pt',2));
         cov_p3 = zeros(2,2);
         for jj = 1:size(pt',2)
             cov_p3 = cov_p3 + dif3(:,jj)*dif3(:,jj)'*tmp_map3(pt(jj,1),pt(jj,2));
         end
         rbt_cent.pdf_cov_cent{count} = cov_p3;
         rbt_cent.pdf_norm_cent(count) = norm(cov_p3,'fro');
         
         % Entropy of posterior pdf
         %
         for i=1:NumOfRobot
             % DBF
             tmp_map1 = rbt(i).map;
             % this avoids the error when some grid has zeros probability
             tmp_map1(tmp_map1 <= realmin) = realmin;
             dis_entropy = -(tmp_map1).*log2(tmp_map1); % get the p*log(p) for all grid points
             rbt(i).entropy(count) = sum(sum(dis_entropy));
             
             % concensus
             tmp_map2 = rbt_cons(i).map;
             % this avoids the error when some grid has zeros probability
             tmp_map2(tmp_map2 <= realmin) = realmin;
             dis_entropy = -(tmp_map2).*log2(tmp_map2); % get the p*log(p) for all grid points
             rbt_cons(i).entropy(count) = sum(sum(dis_entropy));
         end
         % centralized
         tmp_map3 = rbt_cent.map;
         % this avoids the error when some grid has zeros probability
         tmp_map3(tmp_map3 <= realmin) = realmin;
         dis_entropy = -(tmp_map3).*log2(tmp_map3); % get the p*log(p) for all grid points
         rbt_cent.entropy(count) = sum(sum(dis_entropy));
         
        end
        
    
        function plotSim(sim)
            %% %%%%%%%%%%%%%% Plotting for simulation process %%%%%%%%%%%%%%%%%
            
            %% LIFO-DBF
            %
            % plot figures for selected robots
            for k = sim_r_idx
                fig_cnt = fig_cnt+1;
                tmp_hd = figure (fig_cnt); % handle for plot of a single robot's target PDF
                clf(tmp_hd);
                shading interp
                contourf((rbt(k).map)','LineColor','none');
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
                    
                    % draw traget trajectory
                    line_hdl = line(fld.traj(1,:), fld.traj(2,:));
                    set(line_hdl,'Marker','.','Color','k','MarkerSize',3,'LineWidth',2);
                    plot(fld.tx, fld.ty, 'k+','MarkerSize',25,'LineWidth',3);
                    set(gca,'fontsize',30)
                end
                xlabel(['Step=',num2str(count)],'FontSize',30);
            end
            %}
            
            %% Consensus
            % plot figures for selected robots
            %
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
            %
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
    end
end