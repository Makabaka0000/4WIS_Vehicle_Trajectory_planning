function clo= clothoid(Rmin,point)
    N = 50; %% 
    sk = 50; %% 曲线长
    c = 1 / Rmin / sk; %% 曲率变化率，即上文中的曲率变化率 a
    x = zeros(1,N);
    y = zeros(1,N);
    start_x=point.x;
    start_y=point.y;
    for sk_i = sk / N : sk / N : sk
        result_x = start_x; %% 初始化x坐标点的值
        result_y = start_y; %% 初始化y坐标点的值
        for i = 0 : N %% 计算x
            result_x = result_x + ((-1) ^ i) * (c ^ (2 * i)) * (sk_i ^ (4 * i + 1)) / ((2 ^ (2 * i)) * (4 * i + 1) * fctorial(2 * i));
        end
        x(int32(sk_i / sk * N)) = result_x;
        for i = 0 : N %% 计算y
            result_y = result_y + ((-1) ^ i) * (c ^ (2 * i + 1)) * (sk_i ^ (4 * i + 3)) / ((2 ^ (2 * i + 1)) * (4 * i + 3) * fctorial(2 * i + 1));
        end
        y(int32(sk_i / sk * N)) = result_y;
    end
%     x=[start_x,x];
%     y=[start_y,y];
%     plot(x,y,'LineWidth',2,'Color','blue');
%     hold on;
%     plot(x(end),y(end),'*'); %%回旋曲线的最后一个点
%     clo.x=x;
%     clo.y=y;
    rx0=start_x;
    ry0=start_y;
    rotate_x=[];
    rotate_y=[];
    angle=(point.theta+0.0001)*57.3;
    angle=pi/180*angle;
    for ii=1:length(y)
        x0=(x(ii)-rx0)*cos(angle)-(y(ii)-ry0)*sin(angle)+rx0;
        y0=(x(ii)-rx0)*sin(angle)+(y(ii)-ry0)*cos(angle)+ry0;
        rotate_x=[rotate_x,x0];
        rotate_y=[rotate_y,y0];
    end
    clo.x=rotate_x;
    clo.y=rotate_y;
    % plot(rotate_x,rotate_y,'LineWidth',2,'Color','green');
end

function y = fctorial(x)
    n = 1;
    for i = 1 : x
        n = n * i;
    end
    y = n ;
end

