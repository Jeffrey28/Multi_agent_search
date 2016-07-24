% set up for the main program

%% %%%%%%% General Setup %%%%%%%%%%
save_data = false; % save data or not
show_plot = true; % draw plots or not

sim_len = 50; % max step
% rounds of consensus at each time step
cons_step=100;

% select the motion of agents and target
selection = 2;
switch selection
    case 1,  r_move= 0; tar_move=0;
    case 2,  r_move= 0; tar_move=1;
    case 3,  r_move= 1; tar_move=0;
    case 4,  r_move= 1; tar_move=1;
    otherwise, error('No selection.');
end

% the set of robots whose pdf will be drawn
if r_move == 0
    sim_r_idx = [1,3,5];
else
    sim_r_idx = [2,4,6];
end

num_robot = 6;

dt = 1; % discretization time interval

fld_size = [100;100];  % Field size

filter_type = [0 0 0]; % 1: turn on the corresponding filters. [dbf,cons,cent]

%% define target and robot positions
% used for initialization, don't need to call again unless necessary
%{
% robot initialization
rbt(NumOfRobot) = struct;
% these are randomly generated initial positions of robots
% x_set = [20,40,60,80,60,40];
% y_set = [50,20,20,50,80,80];

% generate random initial positions of robots
for ii = 1:NumOfRobot
    rnd_rbt_pos(1,:) = randi([5,fld.x-5],1,trial_num);
    rnd_rbt_pos(2,:) = randi([5,fld.y-5],1,trial_num);
    rbt(ii).init_pos = rnd_rbt_pos;
end

% generate the covariance matrices and offset of sensors
for ii = 1:NumOfRobot
    rbt(ii).sen_cov = randCov(2,fld.x,fld.y); % covariance of Gaussian distribution
    rbt(ii).inv_sen_cov = eye(2)/rbt(ii).sen_cov; % inverse of covariance
    rbt(ii).sen_offset = randn(2,1).*[5;5]; % offset of the center of Gaussian distribution
end

save('sensor_spec.mat','rbt');


% target initialization

% generate random target position (just generate once and use them afterwards)
% rnd_tar_pos(1,:) = randi([5,fld.x-5],1,trial_num);
% rnd_tar_pos(2,:) = randi([5,fld.y-5],1,trial_num);
%}

% these are randomly generated sensor specifications
% load the variable using the name that I want
rbt_spec = load('rbt_spec.mat');
rbt_spec = rbt_spec.rbt;

% these are randomly generated target positions
tx_set = [68, 55, 41, 10, 75, 35, 60, 72, 14, 16];
ty_set = [55, 49, 86, 77, 71, 9, 11, 13, 77, 90];

% communication neighbor
rbt_nbhd = {[2,6],[1,3],[2,4],[3,5],[4,6],[1,5]};

% Setup for multiple trials
trial_num = 1; % 10 % number of trials to run

mode_num = 1;%2;

%% Motion model
% precalculate the prediction matrix so that we don't need to re-compute at
% each iteration.

% target motion model
u_set = [zeros(2,1),ones(2,1),-ones(2,1)]; %inPara.u_set;
V_set = {0.01*eye(2),0.01*eye(2)}; %inPara.V_set;

% Compute the transition matrix for prediction step

% [ptx,pty] = meshgrid(1:fld_size(1),1:fld_size(2));
% pt = [ptx(:),pty(:)];
% upd_matrix = cell(mode_num,1); % pred matrix for all motion models
% 
% for mode_cnt = 1:mode_num
%     % tmp_matrix(ii,:) is the transition probability P(x^i_k+1|x^j_k) for
%     % all x^j_k in the grid
%     tmp_matrix = zeros(size(pt,1),size(pt,1));
%     for ii = 1:size(pt,1)
%         display(ii)
%         % transition matrix
%         % tmp_trans(x,y) shows the transition probability P(x^i_k+1|[x;y]),
%         % considering the dynamic model of vehicle
%         tmp_trans = zeros(fld_size(1),fld_size(2));
% %         mu = pt(ii,:)'+u_set(:,ii);
%         for x = 1:fld_size(1)
%             for y = 1:fld_size(2)
%                 mu = [x;y]+u_set(:,mode_cnt);
%                 tmp_trans(x,y) = mvncdf([pt(ii,1)-0.5;pt(ii,2)-0.5],[pt(ii,1)+0.5;pt(ii,2)+0.5],mu,V_set{mode_cnt});
%             end
%         end
%         tmp_matrix(ii,:) = tmp_trans(:);
%     end
%     upd_matrix{mode_cnt} = tmp_matrix;
% end

% save('upd_matrix.mat','upd_matrix')
if tar_move == 1
    load('upd_matrix.mat','upd_matrix')
else
    % this is not necessary for final program. However, for debugging
    % purpose, I need this identity matrix.
    upd_matrix = eye(fld.fld_size(1)*fld.fld_size(2));
end

%             for ii = 1:size(pt,1)
%                 for jj = 1:size(fld.target.dx_set,2)
%                     %             fld.target.dx = fld.target.dx_set(jj);
%                     %             fld.target.dy = fld.target.dy_set(jj);
%                     tmp_dx = fld.target.dx_set(jj);
%                     tmp_dy = fld.target.dy_set(jj);
%
%                     upd_cell1{ii,jj} = zeros(fld.x,fld.y);
%                     if (pt(ii,1)+fld.target.speed*tmp_dx <= fld.x) && (pt(ii,2)+fld.target.speed*tmp_dy <= fld.y) && (pt(ii,1)+fld.target.speed*tmp_dx >= 1) && (pt(ii,2)+fld.target.speed*tmp_dy >= 1)
%                         upd_cell1{ii,jj}(pt(ii,1)+fld.target.speed*tmp_dx,pt(ii,2)+fld.target.speed*tmp_dy) = 1;
%                     end
%                 end
%             end
            

%% Compute the sensor probility matrix
% just a placeholder now, will see how fast the program will run without
% this precomputation