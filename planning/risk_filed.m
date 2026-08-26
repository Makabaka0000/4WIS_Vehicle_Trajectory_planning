d_safe = 1.0;
sigma_r = 0.5;

% 构造行人预测分布（示例3个）
pedestrians(1).mu = [5;-12];
pedestrians(1).cov = [0.25, 0; 0, 0.25];

pedestrians(2).mu = [5.5054; -11.4973];
pedestrians(2).cov = [0.3068, 0.0; 0, 0.3068];

pedestrians(3).mu = [6.0179; -10.9910];
pedestrians(3).cov = [0.3696, 0; 0, 0.3694];

pedestrians(4).mu = [6.5377; -10.4812];
pedestrians(4).cov = [0.4383,0; 0, 0.4380];

pedestrians(5).mu = [7.0646; -9.9677];
pedestrians(5).cov = [0.5131, 0; 0, 0.51244];

pedestrians(6).mu = [7.5987; -9.4507];
pedestrians(6).cov = [0.5940, 0; 0, 0.5928];

pedestrians(7).mu = [8.1400; -8.9300];
pedestrians(7).cov = [0.6809, 0; 0, 0.6791];

pedestrians(8).mu = [8.6884; -8.4058];
pedestrians(8).cov = [0.7740, 0; 0, 0.7714];

pedestrians(9).mu = [9.2441; -7.8780];
pedestrians(9).cov = [0.8733, 0; 0, 0.8697];

pedestrians(10).mu = [-7; -5];
pedestrians(10).cov = [0.16, 0; 0, 0.09];

pedestrians(11).mu = [-6.5919; -4.6973];
pedestrians(11).cov = [0.1749, 0; 0, 0.1129];

pedestrians(12).mu = [-6.1731; -4.3910];
pedestrians(12).cov = [0.1907, 0; 0, 0.13384];

pedestrians(13).mu = [-5.7435; -4.0812];
pedestrians(13).cov = [0.2075, 0; 0, 0.1667];

pedestrians(14).mu = [-5.3031; -3.7677];
pedestrians(14).cov = [0.2254, 0; 0, 0.1976];

pedestrians(15).mu = [-4.8520; -3.4507];
pedestrians(15).cov = [0.2443, 0; 0, 0.2313];

pedestrians(16).mu = [-4.3900; -3.1300];
pedestrians(16).cov = [0.2644, 0; 0, 0.2678];

pedestrians(17).mu = [-3.9174; -2.8058];
pedestrians(17).cov = [0.2856, 0; 0, 0.3071];

pedestrians(18).mu = [-3.4339; -2.4780];
pedestrians(18).cov = [0.3080, 0; 0, 0.3492];


% 定义绘图区域范围
x_range = -20:0.05:20;
y_range = -20:0.05:20;
[X, Y] = meshgrid(x_range, y_range);
Z = zeros(size(X));  % 存储势能值

% 逐点计算势能
for i = 1:size(X, 1)
    for j = 1:size(X, 2)
        Z(i, j) = compute_total_risk(X(i,j), Y(i,j), pedestrians, d_safe, sigma_r);
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


function U_total = compute_total_risk(x, y, pedestrians, d_safe, sigma_r)
% 输入：
%   x, y         —— 当前车辆位置
%   pedestrians  —— 行人数组，每个包含 .mu (2x1 均值) 和 .cov (2x2 协方差)
%   d_safe       —— 安全距离阈值
%   sigma_r      —— 衰减控制参数
%
% 输出：
%   U_total      —— 车辆当前位置 (x,y) 的总风险势能值

    N = length(pedestrians);  % 行人数量
    U_total = 0;

    for i = 1:N
        mu_i = pedestrians(i).mu;        % [x_i; y_i]
        cov_i = pedestrians(i).cov;      % 2x2 协方差矩阵

        r_i = [x; y] - mu_i;
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
