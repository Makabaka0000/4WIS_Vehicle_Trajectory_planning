clear;
clc;
lw=2.8;
lf=0.96;
lr=0.929;
lb=1.942;
dis1=[];
length_=0;
load('Origin_with.mat');
% load('aadptive.mat');
x=params_.limo.x;
y=params_.limo.y;
w=params_.limo.w;
phy=params_.limo.phy;
theta=params_.limo.theta;
a=params_.limo.a;
v=params_.limo.v;
terminal_time=params_.limo.terminal_time;

before.x=x;
before.y=y;
before.w=w;
before.phy=phy;
before.theta=theta;
before.a=a;
before.v=v;
before.terminal_time=terminal_time;
% before.computation_time=computation_time;
for i=1:length(x)-1
    length_=length_+hypot(x(i+1)-x(i),y(i+1)-y(i));
end


% len=length(obstacles);
% ob_x=[];
% ob_y=[];
%     for kk=1:len-1
%         ob_x=[ob_x,obstacles{1,kk}.x];
%         ob_y=[ob_y,obstacles{1,kk}.y];
%     end
%     dis_before=99999999;
%     for s=1:length(before.x)
%         x=before.x(s);
%         y=before.y(s);
%         theta=before.theta(s);
%         four_corner_x=[x+(lw+lf)*cos(theta)-0.5*lb*sin(theta),x+(lw+lf)*cos(theta)+0.5*lb*sin(theta),...
%             x-lr*cos(theta)+0.5*lb*sin(theta),x-lr*cos(theta)-0.5*lb*sin(theta)];
%         four_corner_y=[y+(lw+lf)*sin(theta)+0.5*lb*cos(theta),y+(lw+lf)*sin(theta)-0.5*lb*cos(theta),...
%             y-lr*sin(theta)-0.5*lb*cos(theta),y-lr*sin(theta)+0.5*lb*cos(theta)];
%         for t=1:length(ob_x)
%             for q=1:4
%                 obstacle_distance_before=hypot(four_corner_x(q)-ob_x(t), four_corner_y(q)-ob_y(t));
%                 x_distance_before=min(20+four_corner_x(q),20-four_corner_x(q));
%                 y_distance_before=min(20+four_corner_y(q),20-four_corner_y(q));
%                 cormin_before=min(x_distance_before,y_distance_before);
%                 dis_before=min(dis_before,min(cormin_before,obstacle_distance_before));
%             end
%         end
%    dis1=[dis1,dis_before];     
%     end  
% dis1=[dis1,dis_before];
    
%     jerk:去掉最大值和最小值，然后计算平均值
jerk1=zeros(1,length(before.x)-1);
dierta_t=before.terminal_time/(length(before.x)-1);
    for u=2:length(a)
        jerk1(u-1)=(before.a(u)-before.a(u-1))/dierta_t;
    end

jerk1_ave=sum(abs(jerk1))/length(jerk1);
max_jerk=max(jerk1(1,:));
% dis_min=min(dis1(1,:));
% dis_ave=sum(dis1)/length(dis1);



