clear;
clc;
load('Org.mat');
% load('Origin_without.mat')
% load('Origin_with.mat')
%先是不考虑碰撞风险的行车走廊
x=params_.ha_result.x;
y=params_.ha_result.y;
theta=params_.ha_result.theta;
xr=x+params_.vehicle.r2p.*cos(theta);
yr=y+params_.vehicle.r2p.*sin(theta);
xf=x+params_.vehicle.f2p.*cos(theta);
yf=y+params_.vehicle.f2p.*sin(theta);
ConstructSafeTravelCorridors(xr,yr,'rear');
ConstructSafeTravelCorridors(xf,yf,'fron');
%考虑碰撞风险，计算风险值,假设行人是随机时间开始运动,封装：给定向量起点，速度，偏差，返回分布1×n规模的
x0=[15,-2,-7];bia_x0=[0.5,0.3,0.4];
y0=[-7,9,-5];bia_y0=[0.5,0.4,0.3];
v_x0=[-0.5,-0.3,0.4];bia_vx0=[0.3,0.2,0.1];
v_y0=[0.5,-0.5,0.3];bia_vy0=[0.3,0.1,0.2];
a_x0=[0.02,0.02,0.03];bia_ax0=[0.01,0.02,0.03];
a_y0=[0.01,0.01,0.01];bia_ay0=[0.005,0.01,0.01];
predict_time=10;
distribution=Caculatedistribution(x0,bia_x0,y0,bia_y0,v_x0,bia_vx0,v_y0,bia_vy0,a_x0,bia_ax0,a_y0,bia_ay0,predict_time);
DrawParkingScenario();
hold on;
for j=1:size(distribution,1)
    for i=1:size(distribution(j).x,2)
        plot(distribution(j,1).x(:,i).mu,distribution(j,1).y(:,i).mu,'r*');
        hold on;
    end
end
plot(params_p.limo.x,params_p.limo.y,'LineWidth',2,'color','g');
hold on
plot(params_.limo.x,params_.limo.y,'LineWidth',2,'color','b');
% hold on
% plot(params_without.limo.x,params_without.limo.y,'LineWidth',2,'color','y');
% title('时间速度曲线');
% legend('概率行车走廊','原方法', 'Location', 'best');

% rear_stc=params_.opti.stc.rear_stc;
% for i=1:size(rear_stc,1)
%     xmin=rear_stc(i,1);
%     xmax=rear_stc(i,2);
%     ymin=rear_stc(i,3);
%     ymax=rear_stc(i,4);
%     x=[xmin,xmin,xmax,xmax,xmin];
%     y=[ymin,ymax,ymax,ymin,ymin];
%     plot(x,y);
%     hold on;
% end

% for i=1:size(modify_rear_stc,1)
%     xmin=modify_rear_stc(i,1);
%     xmax=modify_rear_stc(i,2);
%     ymin=modify_rear_stc(i,3);
%     ymax=modify_rear_stc(i,4);
%     x=[xmin,xmin,xmax,xmax,xmin];
%     y=[ymin,ymax,ymax,ymin,ymin];
%     plot(x,y);
%     hold on;
% end
%修正行车走廊，并选择发生膨胀概率符合要求的，在运动前只考虑障碍物起点的风险分布，在障碍
%开始运动后一一对应，当运动到终点停止后，考虑在终点的风险分布
rear_stc=params_.opti.stc.rear_stc;
front_stc=params_.opti.stc.front_stc;
modify_rear_stc=modifydrvingcorridor(rear_stc,distribution);
modify_front_stc=modifydrvingcorridor(front_stc,distribution);

% % pdf_value=caculate_pdf(x_t,y_t,config);
% pdf_value=pdf(x_t(:,5),config(:,1))*pdf(y_t(:,5),config(:,2))*2.8*1.92;%风险值计算的是一个der_t的风险值
% DrawParkingScenario();
% hold on;
% plot(params_.ha_result.x,params_.ha_result.y);
function [x_it,y_it]=formulate_normal(x0,y0,v_x0,v_y0,a_x0,a_y0,i,der_t)
    u_x=x0.mu+(i/der_t)*((a_x0.mu*der_t^2)/2+der_t*(v_x0.mu+i*a_x0.mu*der_t));
    sigama_x=x0.sigma+(i/der_t)*((a_x0.sigma*der_t^4)/4+der_t^2*(v_x0.sigma+i*a_x0.sigma*der_t^2));
    u_y=y0.mu+(i/der_t)*((a_y0.mu*der_t^2)/2+der_t*(v_y0.mu+i*a_y0.mu*der_t));
    sigama_y=y0.sigma+(i/der_t)*((a_y0.sigma*der_t^4)/4+der_t^2*(v_y0.sigma+i*a_y0.sigma*der_t^2));
    x_it=makedist('Normal','mu',u_x,'sigma',sigama_x);
    y_it=makedist('Normal','mu',u_y,'sigma',sigama_y);
end

function modify_stc=modifydrvingcorridor(stc,distribution)
    %新问题:driving corridor怎么根据风险值划分，以及如何确定一个概率driving corridors
    %尝试的方法:
    %1.先不考虑运动的障碍物，等行车走廊构建完毕后，在走廊的四条边上分别采样得到点，然后逐步缩小框的范围
    %存在问题：假设运动点不在边附近，而是在框里面，那么就会造成误判
    %思路:先确定运动点是否在走廊内，用运动点的尺寸先给出一个粗糙的范围，然后再逐步缩小
    global params_
    modify_stc=[];
    sample_point_num=10;
    length=params_.vehicle.length;
    width=params_.vehicle.lb;
    der_s=params_.opti.der_s;
    for i=1:size(stc,1)
        left=stc(i,1);right=stc(i,2);down=stc(i,3);up=stc(i,4);
        istrue=zeros(1,4);
        shrink_size=zeros(1,4);
        risk_distribution_x=[];
        risk_distribution_y=[];
        for j=1:size(distribution,1)
            if i< distribution(j,1).start_time
                risk_distribution_x=[risk_distribution_x,distribution(j,1).x(:,1)];
                risk_distribution_y=[risk_distribution_y,distribution(j,1).y(:,1)];
            elseif (i>=distribution(j,1).start_time && i<=distribution(j,1).expire_time)
                risk_distribution_x=[risk_distribution_x,distribution(j,1).x(:,i+1-distribution(j,1).start_time)];
                risk_distribution_y=[risk_distribution_y,distribution(j,1).y(:,i+1-distribution(j,1).start_time)];     
            elseif (i>distribution(j,1).expire_time)
                risk_distribution_x=[risk_distribution_x,distribution(j,1).x(:,end)];
                risk_distribution_y=[risk_distribution_y,distribution(j,1).y(:,end)];                    
            end
        end
        mid_x=(left+right)/2;mid_y=(up+down)/2;
        for k=1:size(risk_distribution_x,2)
            if ((risk_distribution_x(:,k).mu-left)*(risk_distribution_x(:,k).mu-right)<0)&&((risk_distribution_y(:,k).mu-up)*(risk_distribution_y(:,k).mu-down)<0)
                if (risk_distribution_x(:,k).mu-mid_x)>0 && (risk_distribution_y(:,k).mu-mid_y)>0
                    right=risk_distribution_x(:,k).mu;
                    up=risk_distribution_y(:,k).mu;
                elseif (risk_distribution_x(:,k).mu-mid_x)>0 && (risk_distribution_y(:,k).mu-mid_y)<0
                    right=risk_distribution_x(:,k).mu;
                    down=risk_distribution_y(:,k).mu;           
                elseif (risk_distribution_x(:,k).mu-mid_x)<0 && (risk_distribution_y(:,k).mu-mid_y)>0
                    left=risk_distribution_x(:,k).mu;
                    up=risk_distribution_y(:,k).mu;            
                elseif (risk_distribution_x(:,k).mu-mid_x)<0 && (risk_distribution_y(:,k).mu-mid_y)<0
                    left=risk_distribution_x(:,k).mu;
                    down=risk_distribution_y(:,k).mu;          
                end
            end
        end
        while sum(istrue)<4
            left=left+shrink_size(:,4)*der_s;right=right-shrink_size(:,2)*der_s;
            down=down+shrink_size(:,3)*der_s;up=up-shrink_size(:,1)*der_s;
            shrink_size=zeros(1,4);
            for j=1:2
                if j==1 
                    interpolation=linspace(left,right,sample_point_num);
                    up_sample_point=[];
                    down_sample_point=[];
                    %对行车走廊边上的点进行采样
                    for k=1:sample_point_num
                        up_sample_point=[up_sample_point;interpolation(1,k),up];
                        down_sample_point=[down_sample_point;interpolation(1,k),down];
                    end
                    %计算风险,依据上一步采样得到的点
    %                 x=params_.ha_result.x;
    %                 y=parmas_.ha_result.y;
    %                 theta=parmas_.ha_result.theta;
    %                 vehicle_sample_point=vehiclesample(x(i),y(i),theta(i));
                    if ~istrue(:,1)
                        for k=1:size(up_sample_point,1)
                            for z=1:size(risk_distribution_x)
                                collision_probability=pdf(risk_distribution_x(:,z),up_sample_point(k,1))*pdf(risk_distribution_y(:,z),up_sample_point(k,2))*length*width;
                                if collision_probability>params_.opti.collision_probability_threshold
                                    shrink_size(:,1)=1;
                                    istrue(:,1)=0;
                                    break;
                                end
                            end
                            istrue(:,1)=1;
                        end  
                    end 
                    if ~istrue(:,3)
                        for k=1:size(down_sample_point,1)
                            for z=1:size(risk_distribution_x)
                                collision_probability=pdf(risk_distribution_x(:,z),down_sample_point(k,1))*pdf(risk_distribution_y(:,z),down_sample_point(k,2))*length*width;
                                if collision_probability>params_.opti.collision_probability_threshold
                                    shrink_size(:,3)=1;
                                    istrue(:,3)=0;
                                    break;
                                end
                            end
                            istrue(:,3)=1;                            
                        end  
                    end
                elseif j==2 
                    interpolation=linspace(down,up,sample_point_num);
                    right_sample_point=[];
                    left_sample_point=[];
                    for k=1:sample_point_num
                        right_sample_point=[right_sample_point;right,interpolation(1,k)];
                        left_sample_point=[left_sample_point;left,interpolation(1,k)];
                    end
                    if ~istrue(:,2)
                        for k=1:size(right_sample_point,1)      
                            for z=1:size(risk_distribution_x)
                                collision_probability=pdf(risk_distribution_x(:,z),right_sample_point(k,1))*pdf(risk_distribution_y(:,z),right_sample_point(k,2))*length*width;
                                if collision_probability>params_.opti.collision_probability_threshold
                                    shrink_size(:,2)=1;
                                    istrue(:,2)=0;
                                    break;
                                end
                            end
                            istrue(:,2)=1;
                        end  
                    end 
                    if ~istrue(:,4)
                        for k=1:size(left_sample_point,1)
                            for z=1:size(risk_distribution_x)
                                collision_probability=pdf(risk_distribution_x(:,z),left_sample_point(k,1))*pdf(risk_distribution_y(:,z),left_sample_point(k,2))*length*width;
                                if collision_probability>params_.opti.collision_probability_threshold
                                    shrink_size(:,4)=1;
                                    istrue(:,4)=0;
                                    break;
                                end
                            end
                            istrue(:,4)=1;
                        end  
                    end
                end
            end   
        end
        modify_stc=[modify_stc;left,right,down,up];
    end
end

function total_distribution=Caculatedistribution(x,bia_x,y,bia_y,v_x,bia_vx,v_y,bia_vy,a_x,bia_ax,a_y,bia_ay,predict_time)
    global params_
    total_distribution=[];
    start_time=[randi([1,params_.opti.nfe/5]),randi([32,38]),randi([35,41])];
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
        for i=0:predict_time
            [x_it,y_it]=formulate_normal(x0,y0,v_x0,v_y0,a_x0,a_y0,i,der_t);
            x_t=[x_t,x_it];
            y_t=[y_t,y_it];
        end 
        distribution.x=x_t;
        distribution.y=y_t;
        distribution.start_time=start_time(:,j);
        distribution.expire_time=expire_time;       
        total_distribution=[total_distribution;distribution];
        j=j+1;
    end
end



function vehicle_sample_point=vehiclesample(x,y,theta)
    global parmas_
    a=parmas_.vehicle.length/2;
    b=parmas_.vehicle.lb/2;
    center_point.x=x;
    center_point.y=y;
    left_up_point.x=x-b*sin(theta)+a*cos(theta);
    left_up_point.y=y+b*cos(theta)+a*sin(theta);
    
    right_up_point.x=x+b*sin(theta)+a*cos(theta);
    right_up_point.y=y-b*cos(theta)+a*sin(theta);
    
    left_down_point.x=x-b*sin(theta)-a*cos(theta);
    left_down_point.y=y+b*cos(theta)-a*sin(theta);
    
    right_down_point.x=x+b*sin(theta)-a*cos(theta);
    right_down_point.y=y-b*cos(theta)-a*sin(theta);
    vehicle_sample_point=[center_point,left_up_point,right_up_point,left_down_point,right_down_point];
end













