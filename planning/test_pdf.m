% 定义二维正态分布的均值和协方差矩阵
mu = [0, 0];       % 均值
sigma = [1, 0; 0, 1];  % 协方差矩阵

% 创建网格坐标
x = -20:0.05:20;
y = -20:0.05:20;
[X, Y] = meshgrid(x, y);

% 计算每个点的二维正态分布的PDF值
Z = mvnpdf([X(:), Y(:)], mu, sigma);
Z = reshape(Z, length(x), length(y));  % 将结果重塑为矩阵形式

% 使用imagesc绘制热力图
imagesc(x, y, Z);
colorbar; % 显示颜色条
axis equal;
xlabel('X');
ylabel('Y');
title('2D Normal Distribution Heatmap');