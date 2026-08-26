function [IsGearShift,gsp_point]=GetMapInformation()
    global params_
    [x,y,~]=SearchAStarPath([params_.task.x0,params_.task.y0,params_.task.theta0],[params_.task.xf,params_.task.yf,params_.task.thetaf]);
    plot(x,y,'LineWidth',2);
    %记录在终点能够转动的转角范围(-pi/2,pi/2)
    FindSpinAngle();
    %返回的应该是(是否需要中途换挡标志位,区域，轨迹)
    IsGearShift=0;
%     GearShiftArea=[];%GearShiftArea.trajectort.x/y/theta
%     Rmin=params_.vehicle.turning_radius_min;
    %此处的角度计算应该还是以原来路径上的点为准
    theta_forward=caculatethetaforward(x,y);
    theta_backward=caculatethetabackward(x,y);
    %caculate the value of alpha_1 and alpha_2
    theta_forward_begin=theta_forward(2);
    theta_forward_end=theta_forward(end-1);
    theta_start=params_.task.theta0;
    theta_goal=params_.task.thetaf;
    theta_start_diff=cos(abs(theta_start-theta_forward_begin));
    theta_end_diff=cos(abs(theta_goal-theta_forward_end));
    %用cos(theta)的值来表示是否满足要求,分成四种情况讨论，seg_x,seg_y尽可能在中间，将其安排在行车走廊的中间点
    if(theta_start_diff>0 && theta_end_diff>0)%正向开
        IsGearShift=1;
        return
    end
    if(theta_start_diff<0 && theta_end_diff<0)%倒着开
        IsGearShift=2;
        return
    end
    if(theta_start_diff>0 && theta_end_diff<0)
        %Find a vast place and set the guided point, replacing the nearest guided point
        gsp_point=FindGSPSpace(x,y,theta_forward,theta_backward,1);%findGSP要和膨胀障碍物不发生碰撞
        IsGearShift=3;
    end
    if(theta_start_diff<0 && theta_end_diff>0)%倒着开一段距离，找个地方调正方向
        %todo:gsp应该靠近起点
        gsp_point=FindGSPSpace(x,y,theta_forward,theta_backward,0);
        IsGearShift=4;
    end
end

function theta_b=caculatethetabackward(x,y)
    global params_
    x=flip(x);
    y=flip(y);
    theta=x;
    theta(1)=params_.task.thetaf;
    theta(end)=params_.task.theta0;
    for ii=2:size(x,2)-1
        theta(ii)=atan2(y(ii)-y(ii-1),x(ii)-x(ii-1));
        while(theta(ii)>2*pi)
            theta(ii)=theta(ii)-2*pi;
        end
        while(theta(ii)<-2*pi)
            theta(ii)=theta(ii)+2*pi;
        end
    end
%     for z=1:length(theta)
%          Arrow([x(z),y(z)],[ x(z)+cos(theta(z)),y(z)+sin(theta(z))],'Length',16,'BaseAngle',90,'TipAngle',16,'Width',2);   
%     end
    theta_b=flip(theta);
end
function  theta_forward=caculatethetaforward(x,y)
    global params_
    theta=x;
    theta(1)=params_.task.theta0;
    theta(end)=params_.task.thetaf;
    for ii=2:size(x,2)-1
        theta(ii)=atan2(y(ii)-y(ii-1),x(ii)-x(ii-1));
        while(theta(ii)>2*pi)
            theta(ii)=theta(ii)-2*pi;
        end
        while(theta(ii)<-2*pi)
            theta(ii)=theta(ii)+2*pi;
        end
    end
%     for z=1:length(theta)
%          Arrow([x(z),y(z)],[ x(z)+cos(theta(z)),y(z)+sin(theta(z))],'Length',16,'BaseAngle',90,'TipAngle',16,'Width',2);   
%     end
    theta_forward=theta;
end
function gsp_point=FindGSPSpace(x,y,theta_forward,theta_backward,direction)
    area=[];
    localbox_size=zeros(length(x),6);
    begin_k=round(length(x)/3);
    end_k=round(length(x)*2/3);
    for kk=begin_k:end_k
        lb=getlength(x(kk),y(kk));
        localbox_size(kk,:)=[x(kk),y(kk),x(kk)-lb(2),x(kk)+lb(4),y(kk)-lb(3),y(kk)+lb(1)];
        s=(lb(4)+lb(2))*(lb(1)+lb(3));
        area=[area;s,kk];
    end
    row=find(area(:,1)==max(area(:,1)));
    index=area(row,2);
    free_space=index(floor((1+length(index))*3/4));
    xmax=localbox_size(free_space,4);
    ymax=localbox_size(free_space,6);
    xmin=localbox_size(free_space,3);
    ymin=localbox_size(free_space,5);
    xedge=xmax-xmin;
    yedge=ymax-ymin;
    %找到离其最近的两个sample point
    xv=[xmin;xmin;xmax;xmax;xmin];
    yv=[ymin;ymax;ymax;ymin;ymin];
    xq=x';
    yq=y';
    [in,~]=inpolygon(xq,yq,xv,yv);
    plot(xv,yv) % polygon
    hold on
    in_index=find(in==1);
    %此处进行插值计算，start_point and goal_point,还需要把朝向记录一下
    gsp_area=polyshape([xmin xmin xmax xmax],[ymin ymax ymax ymin]);
    L_in=[x(in_index(1)-1) y(in_index(1)-1);x(in_index(1)) y(in_index(1))];
    L_out=[x(in_index(end)) y(in_index(end));x(in_index(end)+1) y(in_index(end)+1)];
    [in1,out1]=intersect(gsp_area,L_in);
    [in2,out2]=intersect(gsp_area,L_out);
    if direction
        In_theta=theta_forward(in_index(1)-1);
        Out_theta=theta_backward(in_index(end));
    else
        In_theta=theta_backward(in_index(1)-1);
        Out_theta=theta_forward(in_index(end));
    end
    if(~isempty(in1))
        In_point=[in1(1,:),In_theta];
        Out_point=[in2(end,:),Out_theta];
    else
        In_point=[out1(1,:),In_theta];
        Out_point=[out2(end,:),Out_theta];
    end
    crucial_judge_point=[xmin,ymin;xmin,ymax;xmax,ymax;xmax,ymin];
    mid_judge_point=[xmin,(ymin+ymax)/2;(xmin+xmax)/2,ymax];
    key_point_flag=-1;
    for i=1:size(crucial_judge_point,1)
        if abs(crucial_judge_point(i,1)-In_point(:,1))<0.01 || abs(crucial_judge_point(i,2)-In_point(:,2))<0.01...
            || abs(crucial_judge_point(i,1)-Out_point(:,1))<0.01 || abs(crucial_judge_point(i,2)-Out_point(:,2))<0.01
            continue;
        else
            key_point_flag=i;
            remain_point=[crucial_judge_point(i,:),0];
            break;
        end
    end
    if key_point_flag==-1 %都不是的情况下检查边的中点
        %todo:还要加上一个选择哪条边上的中点
        for i=1:size(mid_judge_point,1)
            if abs(mid_judge_point(i,1)-In_point(:,1))<0.01 || abs(mid_judge_point(i,2)-In_point(:,2))<0.01 ||...
                    abs(mid_judge_point(i,1)-Out_point(:,1))<0.01 || abs(mid_judge_point(i,2)-Out_point(:,2))<0.01
                remain_point=[mid_judge_point(2,:),1];
                break;
            else
                remain_point=[mid_judge_point(1,:),1];
            end
        end 
    end
    gsp_point=[(remain_point(1,1)+(xmax+xmin)/2)/2,(remain_point(1,2)+(ymax+ymin)/2)/2,(In_theta+Out_theta)/2];
    DrawTrajFootprints(gsp_point(1,1),gsp_point(1,2),gsp_point(1,3));
    Arrow([gsp_point(1,1),gsp_point(1,2)],[gsp_point(1,1)+cos(gsp_point(1,3)),gsp_point(1,2)+sin(gsp_point(1,3))],'Length',16,'BaseAngle',90,'TipAngle',16,'Width',2);
%     todo:判断remian_point与In_point和Out_point的位置关系:左和右
%     In_x=remain_point(1,1)-In_point(1,1);In_y=remain_point(1,2)-In_point(1,2);
%     Out_x=remain_point(1,1)-Out_point(1,1);Out_y=remain_point(1,2)-Out_point(1,2);
%     In_direction=0;Out_direction=0;%0 define as left;1 define as right
%     if In_x>0 && In_y>0  && ~remain_point(1,3)
%         if remain_point(1,3)%说明是中点
%             In_direction=1;
%         else
%             In_direction=1;
%         end
%     elseif In_x<0 && In_y>0
%     
%     elseif In_x<0 && In_y<0
%  
%     elseif
%         
%     end


%   todo:clothoid curves
    %the interacction point of two clothoid curves is the guided point
    %要定义生成clothoid的方向        
%         first_clothoid=clothoid(Rmin,start_point);
%         second_clothoid=clothoid(Rmin,goal_point);
%         plot(first_clothoid.x,first_clothoid.y,'LineWidth',2);
%         hold on
%         plot(second_clothoid.x,second_clothoid.y,'LineWidth',2);
%         [itpoint_x,itpoint_y,~,~] = intersections(first_clothoid.x, first_clothoid.y, second_clothoid.x, second_clothoid.y);
%         center_theta=(start_point.theta+goal_point.theta)/2;    
end
function lb=getlength(xc,yc)
    global params_
    lb=zeros(1,4);
    is_completed=zeros(1,4);
    while(sum(is_completed)<4)
        for ind=1:4
            if(is_completed(ind))
                continue;
            end
            test=lb;
            if(test(ind)+params_.hybrid_astar.FindSpaceder_s>params_.hybrid_astar.FindSpacemaxs)
                is_completed(ind)=1;
                continue;
            end
            der_s=params_.hybrid_astar.FindSpaceder_s;
            if(isValid(xc,yc,test,ind,der_s))
                test(ind)=test(ind)+der_s;
                lb=test;
            else
                is_completed(ind)=1;
            end
        end
    end
end
function flag=isValid(x,y,trail,ind,der_s)
    global params_
    flag=0;
    x_minb=x-trail(2);y_maxb=y+trail(1);
    x_maxb=x+trail(4);y_minb=y-trail(3);
    switch ind
    case 1
        xmax=x_maxb;xmin=x_minb;ymin=y_maxb;ymax=y_maxb+der_s;
    case 2
        xmax=x_minb;xmin=x_minb-der_s;ymin=y_minb;ymax=y_maxb;
    case 3
        xmax=x_maxb;xmin=x_minb;ymin=y_minb-der_s;ymax=y_minb;
    case 4
        xmax=x_maxb+der_s;xmin=x_maxb;ymin=y_minb;ymax=y_maxb;
    otherwise
        return;
    end    
    if((xmax>params_.scenario.xmax)||(xmin<params_.scenario.xmin)||(ymax>params_.scenario.ymax)||(ymin<params_.scenario.ymin))
        return;
    end
    
    point_nums=1000;
    vx = [linspace(xmin, xmax, point_nums), linspace(xmax, xmax, point_nums), linspace(xmax, xmin, point_nums), linspace(xmin, xmin, point_nums)];
    vy = [linspace(ymax, ymax, point_nums), linspace(ymax, ymin, point_nums), linspace(ymin, ymin, point_nums), linspace(ymin, ymax, point_nums)];
    for ii = 1 : params_.obstacle.num_obs
         if (any(inpolygon(vx, vy, params_.obs.old_dilated_obs{ii}.x,params_.obs.old_dilated_obs{ii}.y)))
            return;
         end
    end
    flag=1;
end
function FindSpinAngle()
    global params_
    xf=params_.task.xf;
    yf=params_.task.yf;
    thetaf=params_.task.thetaf;
    ccw_theta=pi/2;
    cw_theta=-pi/2;
    for i=0:pi/24:pi/2 
        if(~IsSpinTrajValid(xf,yf,thetaf+i))
            ccw_theta=i;
            break;
        end
    end
    for j=0:-pi/24:-pi/2 
        if(~IsSpinTrajValid(xf,yf,thetaf+j))
            cw_theta=j;
            break;
        end
    end
    params_.hybrid_astar.spin_theta=[ccw_theta,cw_theta]; 
    DrawTrajFootprints(params_.task.xf,params_.task.yf,params_.task.thetaf+params_.hybrid_astar.spin_theta(1,1));
    DrawTrajFootprints(params_.task.xf,params_.task.yf,params_.task.thetaf+params_.hybrid_astar.spin_theta(1,2));
end
function is_collision_free=IsSpinTrajValid(x,y,theta)
    is_collision_free=0;
    global params_
    xr=x-params_.vehicle.r2p*cos(theta);
    yr=y-params_.vehicle.r2p*sin(theta);
    xf=x+params_.vehicle.f2p*cos(theta);
    yf=y+params_.vehicle.f2p*sin(theta);
    xx=[xr,xf];
    yy=[yr,yf];
    if(any(xx>params_.scenario.xmax-params_.vehicle.dual_disk_radius*1.01))
        return;
    elseif(any(xx<params_.scenario.xmin+params_.vehicle.dual_disk_radius*1.01))
        return;
    elseif(any(yy>params_.scenario.ymax-params_.vehicle.dual_disk_radius*1.01))
        return;
    elseif(any(yy<params_.scenario.ymin+params_.vehicle.dual_disk_radius*1.01))
        return;
    end

    ind_x=floor((xx-params_.scenario.xmin)/params_.hybrid_astar.resolution_dx)+1;
    ind_y=floor((yy-params_.scenario.ymin)/params_.hybrid_astar.resolution_dy)+1;
    if(any(params_.scenario.dilated_map(sub2ind(size(params_.scenario.dilated_map),ind_x,ind_y))))
        return;
    end
    is_collision_free=1;
end