%%%%函数作用：进行等时间步长重采样
function [x0,y0,theta0,v0,delta0,a0,w0,terminal_time] = resample(x,y,theta)
global vehicle_geometrics vehicle_kinematics NLP_
w_max = vehicle_kinematics.w_max;
delta_max = vehicle_kinematics.phy_max;
xx=x(1);
yy=y(1);
tt=theta(1);
for ii=2:length(x)
    if((x(ii)==x(ii-1))&&(y(ii)==y(ii-1)))
        continue;
    end
    xx=[xx,x(ii)];
    yy=[yy,y(ii)];
    tt=[tt,theta(ii)];
end
x=xx;
y=yy;
theta=tt;

if(size(x,2)==1)
    x0=ones(1,NLP_.resample.resample_N).*x;
    y0=ones(1,NLP_.resample.resample_N).*y;
    theta0=ones(1,NLP_.resample.resample_N).*theta;
    v0=zeros(1,NLP_.resample.resample_N);
    a0=zeros(1,NLP_.resample.resample_N);
    delta0=zeros(1,NLP_.resample.resample_N);
    w0=zeros(1,NLP_.resample.resample_N);
    terminal_time=zeros(1,NLP_.resample.resample_N);
    return;
end

Nfe=length(theta);
for ii=2:Nfe
    while(theta(ii)-theta(ii-1)>pi)
        theta(ii)=theta(ii)-2*pi;
    end
    while(theta(ii)-theta(ii-1)<-pi)
        theta(ii)=theta(ii)+2*pi;
    end
end

d_total = Estimatedtotaldistance(x,y);
[s,v,a,terminal_time] = Pontryagin_principle(d_total);
[x_0,y_0,theta_0]=InterpolateGrids(x,y,theta);

%%%等距离离散路径点
local_s=linspace(0,d_total,length(x_0));

[x0,y0,theta0] = Findequaltime(s,local_s,x_0,y_0,theta_0);

N = length(x0);
t_step = terminal_time/(N);
delta0 = zeros(1,N);
w0 = zeros(1,N);
v0 = v;
a0 = a;

for m = 2:(N-1)
    delta0(m) = atan((theta0(m+1)-theta0(m))*vehicle_geometrics.length/(v0(m)*t_step));
    if(delta0(m) > delta_max)
        delta0(m) = delta_max;
    elseif(delta0(m) < -delta_max)
        delta0(m) = -delta_max;
    end 

    w0(m) = (delta0(m+1)-delta0(m))/t_step;
    if(w0(m) > w_max)
        w0(m) = w_max;
    elseif(w0(m) < -w_max)
        w0(m) = -w_max;
    end 
end
index=round(linspace(1,length(x0),NLP_.resample.resample_N));
x0=x0(index);
y0=y0(index);
theta0=theta0(index);
v0=v0(index);
a0=a0(index);
delta0=delta0(index);
w0=w0(index);

for ii=2:length(theta0)
    while(theta0(ii)-theta0(ii-1)>pi)
        theta0(ii)=theta0(ii)-2*pi;
    end
    while(theta0(ii)-theta0(ii-1)<-pi)
        theta0(ii)=theta0(ii)+2*pi;
    end
end

end

%%%等时间步长和等距离匹配
function[x0,y0,theta0] = Findequaltime(s,local_s,x,y,theta)
index = zeros(1,size(s,2));
for m = 1:length(s)
    temp=abs(s(m)-local_s);
    temp=find(temp==min(temp));
    index(m)=temp(end);
end
 x0=x(index);
 y0=y(index);
 theta0=theta(index);
end

%%%构建等距离坐标索引
function[x_full,y_full,theta_full]=InterpolateGrids(x,y,theta)
    x_full=[];
    y_full=[];
    theta_full=[];
    for ii=2:length(x)
        Nsp=round(norm([x(ii)-x(ii-1),y(ii)-y(ii-1)])*200);
        temp=linspace(x(ii-1),x(ii),Nsp);
        x_full=[x_full,temp(1,1:(Nsp-1))];

        temp=linspace(y(ii-1),y(ii),Nsp);
        y_full=[y_full,temp(1,1:(Nsp-1))];

        temp=linspace(theta(ii-1),theta(ii),Nsp);
        theta_full=[theta_full,temp(1,1:(Nsp-1))];
    end
    x_full=[x_full,x(end)];
    y_full=[y_full,y(end)];
    theta_full=[theta_full,theta(end)];
end

%%%估算总路程长度
function d_total = Estimatedtotaldistance(x,y)
num_node_a = length(x);
d_total = 0;
for i = 1:num_node_a-1
    d_i = sqrt((x(i+1)-x(i))^2 + (y(i+1)-y(i))^2);
    d_total = d_total + d_i;
end
end

%%%求最小时间子函数
function[s,v,a,terminal_time]=Pontryagin_principle(total_path)
global vehicle_kinematics NLP_

a_max = vehicle_kinematics.a_max;
v_max = vehicle_kinematics.v_max;

v_start = 0;
v_final = 0;
t_acc = (v_max-v_start)/a_max;
t_dec = (v_max-v_start)/a_max;
d_acc = (v_max^2-v_start^2)/(2*a_max);
d_dec = (v_max^2-v_final^2)/(2*a_max);
d_cruise = total_path - (d_acc+d_dec);

if(d_cruise<=0)
    d_acc = total_path/2;
    d_dec = total_path - d_acc;
    t_acc = sqrt(2*d_acc/a_max);            %%%注意，初速度为0
    t_dec = sqrt(2*d_dec/a_max);
    v_max2 = t_acc*a_max;
    t_cruise = 0;
    terminal_time = t_acc + t_dec + t_cruise;
    Nfe=round(terminal_time/NLP_.ipopt.resample_dt);
    time_line=linspace(0,terminal_time,Nfe);

    time_acc=time_line(find(time_line<=0.5*terminal_time));
    time_dec=time_line(find(time_line>0.5*terminal_time));

    a=[ones(1,size(time_acc,2)).*a_max,ones(1,size(time_dec,2)).*-a_max];

    v_acc=time_acc.*a_max;
    v_dec=v_max2+(time_dec-0.5*terminal_time).*-a_max;
    v=[v_acc,v_dec];

    s_acc=0.5*a_max*(time_acc.^2);
    s_dec=d_acc + v_max2*(time_dec - t_acc) - 0.5*a_max*(time_dec - terminal_time/2).^2;
    s=[s_acc,s_dec];
else
    t_cruise=d_cruise/v_max;
    t_slope=v_max/a_max;
    terminal_time = t_acc + t_dec + t_cruise;
    Nfe=round(terminal_time/NLP_.ipopt.resample_dt);

    time_line=linspace(0,terminal_time,Nfe);
    time_acc=time_line(find(time_line<=t_slope));
    time_dec=time_line(find(time_line>t_slope+t_cruise));
    time_accpluscruise=time_line(find(time_line<=t_slope+t_cruise));
    time_cruise=time_accpluscruise(find(time_accpluscruise>t_slope));

    a=[ones(1,size(time_acc,2)).*a_max,zeros(1,size(time_cruise,2)),ones(1,size(time_dec,2)).*-a_max];

    v_acc=time_acc.*a_max;
    v_crusie=ones(1,size(time_cruise,2)).*v_max;
    v_dec=v_max+(time_dec-t_slope-t_cruise).*-a_max;
    v=[v_acc,v_crusie,v_dec];

    s_acc=0.5*a_max*(time_acc.^2);
    s_cruise=0.5*a_max*(t_slope^2)+v_max*(time_cruise-t_slope);
    s_dec=0.5*a_max*(t_slope^2)+v_max*t_cruise+v_max*(time_dec-t_slope-t_cruise)+0.5*-a_max*((time_dec-t_slope-t_cruise).^2);
    s=[s_acc,s_cruise,s_dec];
end
end

% %%%庞特里亚金预估规划总时间
% function [s,v,a,terminal_time,num_t,time_lines] =  Pontryagin_principle(d_total)
% 
% global vehicle_kinematics NLP_ipopt
% a_max = vehicle_kinematics.a_max;
% v_max = vehicle_kinematics.v_max;
% v_start = 0;
% v_final = 0;
% 
% t_acc = (v_max-v_start)/a_max;
% t_dec = (v_max-v_start)/a_max;
% d_acc = (v_max^2-v_start^2)/(2*a_max);
% d_dec = (v_max^2-v_final^2)/(2*a_max);
% d_cruise = d_total - (d_acc+d_dec);
% 
% if d_cruise < 0                             %%%等加速度、减加速度的路程大于总路程
%     d_acc = d_total/2;
%     d_dec = d_total - d_acc;
%     t_acc = sqrt(2*d_acc/a_max);            %%%注意，初速度为0
%     t_dec = sqrt(2*d_dec/a_max);
%     v_max2 = t_acc*a_max;
%     t_cruise = 0;
%     terminal_time = t_acc + t_dec + t_cruise;
%     num_t = round(terminal_time/NLP_ipopt.resample_dt);
%     time_line = linspace(0,terminal_time,num_t);
%     time_lines = time_line;
%     time_acc=time_line(find(time_line<=0.5*terminal_time));     %加速的时间
%     time_dec=time_line(find(time_line>0.5*terminal_time));      %减速的时间
% 
%     a = [ones(1,size(time_acc,2).*a_max),...
%         ones(1,size(time_dec,2).*-a_max)];
% 
%     v_acc = time_acc.*a_max;
%     v_dec = v_max2 + (time_dec - t_acc).*-a_max;
%     v = [v_acc,v_dec];
% 
%     s_acc = 0.5*a_max*(time_acc.^2);
%     s_dec = d_acc + v_max2*(time_dec - t_acc) - 0.5*a_max*(time_dec - terminal_time/2)^2;
%     s = [s_acc,s_dec];
% else
%     t_cruise = d_cruise/v_max;
%     terminal_time = t_acc + t_dec + t_cruise;
%     num_t = round(terminal_time/NLP_ipopt.resample_dt);
%     time_line = linspace(0,terminal_time,num_t);
%     time_lines = time_line;
%     time_slope = v_max/a_max;
%     time_acc=time_line(find(time_line<=time_slope));                    %加速的时间
%     time_dec=time_line(find(time_line>(time_slope + t_cruise)));        %减速的时间
%     time_accplusdec = time_line(find(time_line<=(time_slope + t_cruise)));
%     time_cruise = time_accplusdec(find(time_accplusdec>time_slope));
% 
%     a = [ones(1,size(time_acc,2).*a_max),zeros(1,size(time_cruise,2)),...
%         ones(1,size(time_dec,2).*-a_max)];
% 
%     v_acc = time_acc.*a_max;
%     v_cruise = ones(1,size(time_cruise,2)).*v_max;
%     v_dec = v_max + (time_dec - t_cruise - t_acc).*-a_max;
%     v = [v_acc,v_cruise,v_dec];
% 
%     s_acc = 0.5*a_max*(time_acc.^2);
%     % s_cruise = d_acc + v_max.*time_cruise;
%     s_dec = d_acc + d_cruise + v_max*(time_dec - t_cruise - t_acc) -...
%         0.5*a_max*(time_dec - t_cruise - t_acc).^2;
%     s = [s_acc,s_dec];
% end
% 
% end