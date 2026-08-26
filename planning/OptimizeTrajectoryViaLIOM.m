function OptimizeTrajectoryViaLIOM()
    global params_
    params_.limo.is_successful=0;
    if(isempty(params_.ha_result.x))
        DrawParkingScenario();
        Arrow([params_.task.x0,params_.task.y0],[params_.task.x0+cos(params_.task.theta0),params_.task.y0+sin(params_.task.theta0)],'Length',16,'BaseAngle',90,'TipAngle',16,'Width',2);
        Arrow([params_.task.xf,params_.task.yf],[params_.task.xf+cos(params_.task.thetaf),params_.task.yf+sin(params_.task.thetaf)],'Length',16,'BaseAngle',90,'TipAngle',16,'Width',2);
        SaveFigure([pwd,'\DemoResults\','LIOM_',num2str(params_.user.case_id),'.tif']);
        return;
    end
    x=params_.ha_result.x;
    y=params_.ha_result.y;
    theta=params_.ha_result.theta;
    v=params_.ha_result.v;
    a=params_.ha_result.a;
    phy=params_.ha_result.phy;
    w=params_.ha_result.w;
    terminal_time=params_.ha_result.terminal_time;
    FormInitialGuessViaFullConfig(x,y,theta,v,a,phy,w,terminal_time);
%     x0=[15,-2,-7];bia_x0=[0.5,0.3,0.4];
%     y0=[-7,9,-5];bia_y0=[0.5,0.4,0.3];
%     v_x0=[-0.5,-0.3,0.4];bia_vx0=[0.3,0.2,0.1];
%     v_y0=[0.5,-0.5,0.3];bia_vy0=[0.3,0.1,0.2];
%     a_x0=[0.02,0.02,0.03];bia_ax0=[0.01,0.02,0.03];
%     a_y0=[0.01,0.01,0.01];bia_ay0=[0.005,0.01,0.01];
%     predict_time=10;
%     distribution=Caculatedistribution(x0,bia_x0,y0,bia_y0,v_x0,bia_vx0,v_y0,bia_vy0,a_x0,bia_ax0,a_y0,bia_ay0,predict_time);
    iter=0;
    while(iter<params_.opti.max_iter)
        iter=iter+1;disp(['LIOM Iter = ',num2str(iter)]);

        xr=x+params_.vehicle.r2p.*cos(theta);
        yr=y+params_.vehicle.r2p.*sin(theta);
        xf=x+params_.vehicle.f2p.*cos(theta);
        yf=y+params_.vehicle.f2p.*sin(theta);
%         ConstructMinLocalbox(xr,yr,'rear');
%         ConstructMinLocalbox(xf,yf,'fron');
%         ConstructMaxLocalbox(xr,yr,'rear');
%         ConstructMaxLocalbox(xf,yf,'fron');
%         WriterearCAC('rear');
%         WritefronCAC('fron');
        
        %普通的行车走廊
%         ConstructSafeTravelCorridors(xr,yr,'rear');
%         ConstructSafeTravelCorridors(xf,yf,'fron');
        %修正行车走廊，并选择发生膨胀概率符合要求的，在运动前只考虑障碍物起点的风险分布，在障碍
        %开始运动后一一对应，当运动到终点停止后，考虑在终点的风险分布
        rear_stc=params_.opti.stc.rear_stc;
        front_stc=params_.opti.stc.front_stc;
        modify_rear_stc=modifydrvingcorridor(rear_stc);
        modify_front_stc=modifydrvingcorridor(front_stc);
        WriteStcConstraints(modify_rear_stc,'rear');
        WriteStcConstraints(modify_front_stc,'fron');
        if(params_.user.demo.enable_gif_stc)
            GenerateGifForStcConstruction(xr,yr,xf,yf,iter);
        end
!ampl rr.run
        [x,y,theta,v,a,phy,w,terminal_time,infeasibility]=LoadAmplSolution();

        if(params_.user.demo.enable_plot_intermediate_optimum)
            if(iter==1)
                DrawParkingScenario();
                plot(params_.ha_result.x,params_.ha_result.y,'r');hold on;drawnow;
            end
            plot(x,y,'k');hold on;drawnow;
        end

        if(infeasibility<params_.opti.acceptance_tolerance)
            params_.limo.is_successful=1;
            params_.limo.x=x;
            params_.limo.y=y;
            params_.limo.theta=theta;
            params_.limo.phy=phy;
            params_.limo.v=v;
            params_.limo.a=a;
            params_.limo.w=w;
            params_.limo.terminal_time=terminal_time;
            break;
        end
    end

    if((params_.user.demo.enable_final_result_tiff_and_figure)&&(params_.limo.is_successful))
        DrawParkingScenario();
        DrawTrajFootprints(params_.limo.x,params_.limo.y,params_.limo.theta);
        plot(params_.limo.x,params_.limo.y,'Color',params_.utility.colorpool(1,:),'LineWidth',2);
        Arrow([params_.task.x0,params_.task.y0],[params_.task.x0+cos(params_.task.theta0),params_.task.y0+sin(params_.task.theta0)],'Length',16,'BaseAngle',90,'TipAngle',16,'Width',2);
        Arrow([params_.task.xf,params_.task.yf],[params_.task.xf+cos(params_.task.thetaf),params_.task.yf+sin(params_.task.thetaf)],'Length',16,'BaseAngle',90,'TipAngle',16,'Width',2);
        SaveFigure([pwd,'\DemoResults\','LIOM_',num2str(params_.user.case_id),'.tif']);
%         savefig([pwd,'\DemoResults\','LIOM_',num2str(params_.user.case_id),'.fig']);
    end

    if(params_.user.demo.enable_gif_liom)
        GenerateGifForLimo();
    end
end

function[x,y,theta,v,a,phy,w,terminal_time,infeasibility]=LoadAmplSolution()
    load([pwd,'\AmplResults\','x.txt']);
    load([pwd,'\AmplResults\','y.txt']);
    load([pwd,'\AmplResults\','theta.txt']);
    load([pwd,'\AmplResults\','v.txt']);
    load([pwd,'\AmplResults\','a.txt']);
    load([pwd,'\AmplResults\','phy.txt']);
    load([pwd,'\AmplResults\','w.txt']);
    load([pwd,'\AmplResults\','terminal_time.txt']);
    load([pwd,'\AmplResults\','infeasibility.txt']);
end

function GenerateGifForStcConstruction(xr,yr,xf,yf,iter)
    if(iter~=1)
        return;
    end
    global params_
    name_str=['STC_',num2str(params_.user.case_id)];
    name_str_gif=[pwd,'\DemoResults\',name_str,'.gif'];
    [I,map]=rgb2ind(frame2im(getframe(gcf)),256);
    imwrite(I,map,name_str_gif,'gif','Loopcount',inf,'DelayTime',0.05);

    close all;
    figure(params_.user.case_id);
    set(0,'DefaultLineLineWidth',1);
    hold on;
    box on;
    grid minor;
    axis equal;
    axis([params_.scenario.xmin,params_.scenario.xmax,params_.scenario.ymin,params_.scenario.ymax]);
    set(gcf,'outerposition',get(0,'screensize'));
    xlabel('x / m','FontSize',17,'FontName','Arial Narrow','FontWeight','Bold');
    ylabel('y / m','FontSize',17,'FontName','Arial Narrow','FontWeight','Bold');

    xmin=params_.scenario.xmin;
    ymin=params_.scenario.ymin;
    resolution_x=params_.hybrid_astar.resolution_dx;
    resolution_y=params_.hybrid_astar.resolution_dy;
    for jj=1:params_.hybrid_astar.num_nodes_x
        for kk=1:params_.hybrid_astar.num_nodes_y
            if(params_.scenario.dilated_map(jj,kk)==1)
                x0=xmin+(jj-1)*resolution_x;
                y0=ymin+(kk-1)*resolution_y;
                x1=xmin+jj*resolution_x;
                y1=ymin+kk*resolution_y;
                vx=[linspace(x0,x1,2),linspace(x1,x1,2),linspace(x1,x0,2),linspace(x0,x0,2)];
                vy=[linspace(y1,y1,2),linspace(y1,y0,2),linspace(y0,y0,2),linspace(y0,y1,2)];
                fill(vx,vy,[.5,.5,.5],'EdgeColor',[0,0,0]);
            end
        end
    end
    for jj=1:params_.obstacle.num_obs
        V=params_.obstacle.obs{jj};
        fill(V.x,V.y,[0.5,0.5,0.5]);
    end
    [I,map]=rgb2ind(frame2im(getframe(gcf)),256);
    imwrite(I,map,name_str_gif,'gif','WriteMode','append','DelayTime',0.05);

    for ii=1:params_.opti.nfe
        plot(xf,yf,'Color',params_.utility.colorpool(1,:),'LineWidth',3);
        plot(xf(1:ii),yf(1:ii),'k.');
        DrawBox(params_.opti.stc.front_stc(ii,:),'fron');drawnow;
        [I,map]=rgb2ind(frame2im(getframe(gcf)),256);
        imwrite(I,map,name_str_gif,'gif','WriteMode','append','DelayTime',0.01);
    end


    for ii=1:params_.opti.nfe
        plot(xr,yr,'Color',params_.utility.colorpool(3,:),'LineWidth',3);
        plot(xr(1:ii),yr(1:ii),'k.');
        DrawBox(params_.opti.stc.rear_stc(ii,:),'rear');drawnow;
        [I,map]=rgb2ind(frame2im(getframe(gcf)),256);
        imwrite(I,map,name_str_gif,'gif','WriteMode','append','DelayTime',0.01);
    end
end

function DrawBox(vec,str_flag)
    xmin=vec(1);
    xmax=vec(2);
    ymin=vec(3);
    ymax=vec(4);
    x=[xmax,xmax,xmin,xmin,xmax];
    y=[ymax,ymin,ymin,ymax,ymax];
    if(all(str_flag=='fron'))
        plot(x,y,'Color',[34,177,76]./255,'LineWidth',0.7);
    elseif(all(str_flag=='rear'))
        plot(x,y,'Color',[0,0,255]./255,'LineWidth',0.7);
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

function modify_stc=modifydrvingcorridor(stc)
    %新问题:driving corridor怎么根据风险值划分，以及如何确定一个概率driving corridors
    %尝试的方法:
    %1.先不考虑运动的障碍物，等行车走廊构建完毕后，在走廊的四条边上分别采样得到点，然后逐步缩小框的范围
    %存在问题：假设运动点不在边附近，而是在框里面，那么就会造成误判
    %思路:先确定运动点是否在走廊内，用运动点的尺寸先给出一个粗糙的范围，然后再逐步缩小
    global params_
    distribution=params_.distribution;
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
function WriteStcConstraints(box_edge_length,filename)
    global params_
    if(all(filename=='fron'))
        params_.opti.stc.front_stc=box_edge_length;
    elseif(all(filename=='rear'))
        params_.opti.stc.rear_stc=box_edge_length;
    end
    delete(['STC_',filename]);
    fid=fopen(['STC_',filename],'w');
    for ii=1:length(box_edge_length)
        for jj=1:4
            fprintf(fid,'%g %g %.6f \r\n',ii,jj,box_edge_length(ii,jj));
        end
    end
    fclose(fid);
end
function CreateCostmaps()
    global params_
    xmin = params_.scenario.xmin;
    ymin = params_.scenario.ymin;
    resolution_x = params_.hybrid_astar.resolution_dx;
    resolution_y = params_.hybrid_astar.resolution_dy;
    costmap = zeros(params_.hybrid_astar.num_nodes_x, params_.hybrid_astar.num_nodes_y);

    for ii = 1 : size(params_.obstacle.obs, 2)
        obs_vx = params_.obstacle.obs{ii}.x;
        obs_vy = params_.obstacle.obs{ii}.y;
        x_lb = min(obs_vx);
        x_ub = max(obs_vx);
        y_lb = min(obs_vy);
        y_ub = max(obs_vy);
        [Nmin_x, Nmin_y] = ConvertXyToId(x_lb, y_lb);
        [Nmax_x, Nmax_y] = ConvertXyToId(x_ub, y_ub);
        Nmin_x = max(Nmin_x - 1, 1);
        Nmin_y = max(Nmin_y - 1, 1);
        Nmax_x = min(Nmax_x + 1, params_.hybrid_astar.num_nodes_x);
        Nmax_y = min(Nmax_y + 1, params_.hybrid_astar.num_nodes_y);
        for jj = Nmin_x : Nmax_x
            for kk = Nmin_y : Nmax_y
                if (costmap(jj, kk) == 1)
                    continue;
                end
                x0 = xmin + (jj - 1) * resolution_x;
                y0 = ymin + (kk - 1) * resolution_y;
                x1 = xmin + jj * resolution_x;
                y1 = ymin + kk * resolution_y;
                vx = [linspace(x0, x1, 5), linspace(x1, x1, 5), linspace(x1, x0, 5), linspace(x0, x0, 5)];
                vy = [linspace(y1, y1, 5), linspace(y1, y0, 5), linspace(y0, y0, 5), linspace(y0, y1, 5)];
                if (any(inpolygon(vx, vy, params_.obstacle.obs{ii}.x, params_.obstacle.obs{ii}.y)))
                    costmap(jj, kk) = 1;
                end
            end
        end
    end
    params_.scenario.original_map = costmap;
    length_unit = 0.5 * (resolution_x + resolution_y);
    basic_elem = strel('disk', 0 + ceil(params_.vehicle.dual_disk_radius / length_unit));
    params_.scenario.dilated_map = imdilate(costmap, basic_elem);
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
