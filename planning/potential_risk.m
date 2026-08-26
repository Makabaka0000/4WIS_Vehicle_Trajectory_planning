clear;
clc;
% load('OgrinDC.mat');
load('Safe4DC.mat');
%先是不考虑碰撞风险的行车走廊
x=params_.limo.x;
y=params_.limo.y;
v=params_.limo.v;
%考虑碰撞风险，计算风险值,假设行人是随机时间开始运动,封装：给定向量起点，速度，偏差，返回分布1×n规模的
x0=[5,-7];bia_x0=[0.5,0.4];
y0=[-12,-5];bia_y0=[0.5,0.3];
v_x0=[0.5,0.4];bia_vx0=[0.3,0.1];
v_y0=[0.5,0.3];bia_vy0=[0.3,0.2];
a_x0=[0.02,0.03];bia_ax0=[0.01,0.03];
a_y0=[0.01,0.01];bia_ay0=[0.005,0.01];
predict_time=8;
distribution=Caculatedistribution(x0,bia_x0,y0,bia_y0,v_x0,bia_vx0,v_y0,bia_vy0,a_x0,bia_ax0,a_y0,bia_ay0,predict_time);


x_range = -20:0.05:20;
y_range = -20:0.05:20;
[X, Y] = meshgrid(x_range, y_range);
Z = zeros(size(X));  % 存储势能值
N=length(distribution);
for k=1:N
pedestrians=distribution(k);
    for i = 1:size(X, 1)
        for j = 1:size(X, 2)
            Z(i, j) =max(Z(i,j),compute_total_risk(X(i,j), Y(i,j), pedestrians));
        end
    end
end

% 可视化热力图
figure;
imagesc(x_range, y_range, Z);
set(gca, 'YDir', 'normal');  % 修正 y 轴方向
colorbar;
xlabel('x (m)');
ylabel('y (m)');
title('Probabilistic Risk Potential Field');


function U_total = compute_total_risk(x, y, pedestrians)
% 输入：
%   x, y         —— 当前车辆位置
%   pedestrians  —— 行人数组，每个包含 .mu (2x1 均值) 和 .cov (2x2 协方差)
%   d_safe       —— 安全距离阈值
%   sigma_r      —— 衰减控制参数
%
% 输出：
%   U_total      —— 车辆当前位置 (x,y) 的总风险势能值
    d_safe = 2.0;
    sigma_r = 0.5;
    N = length(pedestrians);  % 行人数量
    U_total = 0;

    for i = 1:N
        mu_i = pedestrians.state(i).mu;        % [x_i; y_i]
        cov_i = pedestrians.state(i).Sigma;      % 2x2 协方差矩阵

        r_i = [x; y] - mu_i';
        d_M_i = sqrt(r_i' * inv(cov_i) * r_i);  % Mahalanobis 距离

        % 分段势能函数
        if d_M_i <= d_safe
            U_pi = 1;
        else
            U_pi = exp(-((d_M_i - d_safe)^2) / (2 * sigma_r^2));
        end

        U_total = U_total + U_pi;  % 累加每个行人带来的风险势能
    end
end

function [x_it,y_it]=formulate_normal(x0,y0,v_x0,v_y0,a_x0,a_y0,i,der_t)
    u_x=x0.mu+(i/der_t)*((a_x0.mu*der_t^2)/2+der_t*(v_x0.mu+i*a_x0.mu*der_t));
    sigama_x=x0.sigma+(i/der_t)*((a_x0.sigma*der_t^4)/4+der_t^2*(v_x0.sigma+i*a_x0.sigma*der_t^2));
    u_y=y0.mu+(i/der_t)*((a_y0.mu*der_t^2)/2+der_t*(v_y0.mu+i*a_y0.mu*der_t));
    sigama_y=y0.sigma+(i/der_t)*((a_y0.sigma*der_t^4)/4+der_t^2*(v_y0.sigma+i*a_y0.sigma*der_t^2));
    x_it=makedist('Normal','mu',u_x,'sigma',sigama_x);
    y_it=makedist('Normal','mu',u_y,'sigma',sigama_y);
end

function total_distribution=Caculatedistribution(x,bia_x,y,bia_y,v_x,bia_vx,v_y,bia_vy,a_x,bia_ax,a_y,bia_ay,predict_time)
    global params_
    total_distribution=[];
    start_time=[randi([1,params_.opti.nfe/5]),randi([32,38])];
    j=1;
    for i=1:size(x,2)
        x0=makedist('Normal','mu',x(:,i),'sigma',bia_x(:,i));
        y0=makedist('Normal','mu',y(:,i),'sigma',bia_y(:,i));
        v_x0=makedist('Normal','mu',v_x(:,i),'sigma',bia_vx(:,i));
        v_y0=makedist('Normal','mu',v_y(:,i),'sigma',bia_vy(:,i));
        a_x0=makedist('Normal','mu',a_x(:,i),'sigma',bia_ax(:,i));
        a_y0=makedist('Normal','mu',a_y(:,i),'sigma',bia_ay(:,i));
        der_t=params_.ha_result.terminal_time/params_.opti.nfe;%这个der_t要与ig里的采样时间对应，各自运动一个时间间隔，车一直运动，人只运动有限时间
        %每个obs运动的时间要确定下来，可以确定的是，开始运动的时间必须随机
        expire_time=start_time(:,j)+predict_time;
        x_t=x0;
        y_t=y0;
        state_sequence=[];
        for k=0:predict_time
            [x_it,y_it]=formulate_normal(x0,y0,v_x0,v_y0,a_x0,a_y0,k,der_t);
            x_t=[x_t,x_it];
            y_t=[y_t,y_it];
            mu = [x_it.mu, y_it.mu];
            sigma_x = x_it.sigma;
            sigma_y = y_it.sigma;
            rho = 0;  % 假设x, y独立；也可以根据实际模型设置相关性
            Sigma = [sigma_x^2, rho*sigma_x*sigma_y;
                     rho*sigma_x*sigma_y, sigma_y^2];
            xy_dist.mu = mu;
            xy_dist.Sigma = Sigma;
            state_sequence = [state_sequence, xy_dist];
        end 
        distribution.state=state_sequence;
        distribution.start_time=start_time(:,j);
        distribution.expire_time=expire_time;       
        total_distribution=[total_distribution;distribution];
        j=j+1;
    end
end