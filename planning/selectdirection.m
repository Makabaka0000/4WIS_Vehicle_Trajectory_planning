function [seg_x,seg_y,seg_theta,IsGearShift]=selectdirection(keypoint_index,origin_path)
    global params_
    FindSpinAngle();
    IsGearShift=[-1,-1];
    %此处的角度计算应该还是以原来路径上的点为准
    theta_forward=caculatethetaforward(origin_path.x,origin_path.y);
    theta_backward=caculatethetabackward(origin_path.x,origin_path.y);
    %caculate the value of alpha_1 and alpha_2
    theta_forward_begin=theta_forward(2);
    theta_forward_end=theta_forward(end-1);
    theta_start=params_.task.theta0;
    theta_goal=params_.task.thetaf;
    theta_start_diff=cos(abs(theta_start-theta_forward_begin));
    theta_end_diff=cos(abs(theta_goal-theta_forward_end));
    %用cos(theta)的值来表示是否满足要求,分成四种情况讨论，seg_x,seg_y尽可能在中间，将其安排在行车走廊的中间点
    if(theta_start_diff>0 && theta_end_diff>0)%正向开
        IsGearShift(1,1)=1;
%         seg_x=origin_path.x(1,keypoint_index);
%         seg_y=origin_path.y(1,keypoint_index);
        seg_theta=theta_forward(1,keypoint_index);
        [seg_x,seg_y]=ModifyCenterPoint(origin_path.x(1,keypoint_index),origin_path.y(1,keypoint_index),seg_theta);
%         for z=1:length(seg_theta)
%             Arrow([seg_x(z),seg_y(z)],[ seg_x(z)+cos(seg_theta(z)),seg_y(z)+sin(seg_theta(z))],'Length',16,'BaseAngle',90,'TipAngle',16,'Width',2);   
%         end
    end
    if(theta_start_diff<0 && theta_end_diff<0)%倒着开
        IsGearShift(1,1)=2;
%         seg_x=origin_path.x(1,keypoint_index);
%         seg_y=origin_path.y(1,keypoint_index);
        seg_theta=theta_backward(1,keypoint_index);
        [seg_x,seg_y]=ModifyCenterPoint(origin_path.x(1,keypoint_index),origin_path.y(1,keypoint_index),seg_theta);
%         for z=1:length(seg_theta)
%             Arrow([seg_x(z),seg_y(z)],[ seg_x(z)+cos(seg_theta(z)),seg_y(z)+sin(seg_theta(z))],'Length',16,'BaseAngle',90,'TipAngle',16,'Width',2);   
%         end
    end
    if(theta_start_diff>0 && theta_end_diff<0)
        %Find a vast place and set the guided point, replacing the nearest guided point
        IsGearShift(1,1)=3;
        [gsp_point,start_index,goal_index]=FindGSPSpace(origin_path.x,origin_path.y,theta_forward,theta_backward,1);
        point_index_forward=[];
        point_index_backward=[];
        for i=keypoint_index
            if i<start_index
                point_index_forward=[point_index_forward,i];
            elseif i>goal_index
                point_index_backward=[point_index_backward,i];
            end
        end
%         x_f=origin_path.x(point_index_forward);
%         y_f=origin_path.y(point_index_forward);
        theta_f=theta_forward(point_index_forward);
        [x_f,y_f]=ModifyCenterPoint(origin_path.x(point_index_forward),origin_path.y(point_index_forward),theta_f);
%         
%         x_b=origin_path.x(point_index_backward);
%         y_b=origin_path.y(point_index_backward);
        theta_b=theta_backward(point_index_backward);
        [x_b,y_b]=ModifyCenterPoint(origin_path.x(point_index_backward),origin_path.y(point_index_backward),theta_b);
        
        seg_x=x_f;
        IsGearShift(1,2)=size(seg_x,2)+1;
        seg_x=[seg_x,gsp_point(1,1),x_b];
        seg_y=y_f;
        seg_y=[seg_y,gsp_point(1,2),y_b];
        seg_theta=theta_f;
        seg_theta=[seg_theta,gsp_point(1,3),theta_b]; 
%         for z=1:length(seg_theta)
%             Arrow([seg_x(z),seg_y(z)],[ seg_x(z)+cos(seg_theta(z)),seg_y(z)+sin(seg_theta(z))],'Length',16,'BaseAngle',90,'TipAngle',16,'Width',2);   
%         end
    end
    if(theta_start_diff<0 && theta_end_diff>0)%倒着开一段距离，找个地方调正方向
        IsGearShift(1,1)=4;
        [gsp_point,start_index,goal_index]=FindGSPSpace(origin_path.x,origin_path.y,theta_forward,theta_backward,0);
        point_index_backward=[];
        point_index_forward=[];
        for i=keypoint_index
            if i<start_index
                point_index_backward=[point_index_backward,i];
            elseif i>goal_index
                point_index_forward=[point_index_forward,i];
            end
        end
%         x_b=origin_path.x(point_index_backward);
%         y_b=origin_path.y(point_index_backward);
        theta_b=theta_backward(point_index_backward);
        [x_b,y_b]=ModifyCenterPoint(origin_path.x(point_index_backward),origin_path.y(point_index_backward),theta_b);
        
%         x_f=origin_path.x(point_index_forward);
%         y_f=origin_path.y(point_index_forward);
        theta_f=theta_forward(point_index_forward);
        [x_f,y_f]=ModifyCenterPoint(origin_path.x(point_index_forward),origin_path.y(point_index_forward),theta_f);
        
        
        
        seg_x=x_b;
        IsGearShift(1,2)=size(seg_x,2)+1;
        seg_x=[seg_x,gsp_point(1,1),x_f];
        seg_y=y_b;
        seg_y=[seg_y,gsp_point(1,2),y_f];
        seg_theta=theta_b;
        seg_theta=[seg_theta,gsp_point(1,3),theta_f]; 
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
        while(theta(ii)<0)
            theta(ii)=theta(ii)+2*pi;
        end
    end
%     for z=1:length(theta)
%          Arrow([x(z),y(z)],[ x(z)+cos(theta(z)),y(z)+sin(theta(z))],'Length',16,'BaseAngle',90,'TipAngle',16,'Width',2);   
%     end
    theta_forward=theta;
end
function [gsp_point,start_index,goal_index]=FindGSPSpace(x,y,theta_forward,theta_backward,direction)
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
    start_index=in_index(1);
    goal_index=in_index(end);
    
    
    
    
%      gsp_area=polyshape([xmin xmin xmax xmax],[ymin ymax ymax ymin]);
%     L_in=[x(in_index(1)-1) y(in_index(1)-1);x(in_index(1)) y(in_index(1))];
%     L_out=[x(in_index(end)) y(in_index(end));x(in_index(end)+1) y(in_index(end)+1)];
%     [in1,out1]=intersect(gsp_area,L_in);
%     [in2,out2]=intersect(gsp_area,L_out);
    if direction
        In_theta=theta_forward(in_index(1)-1);
        Out_theta=theta_backward(in_index(end));
    else
        In_theta=theta_backward(in_index(1)-1);
        Out_theta=theta_forward(in_index(end));
    end
%     if(~isempty(in1))
%         In_point=[in1(1,:),In_theta];
%         Out_point=[in2(end,:),Out_theta];
%     else
%         In_point=[out1(1,:),In_theta];
%         Out_point=[out2(end,:),Out_theta];
%     end
%     crucial_judge_point=[xmin,ymin;xmin,ymax;xmax,ymax;xmax,ymin];
%     mid_judge_point=[xmin,(ymin+ymax)/2;(xmin+xmax)/2,ymax];
%     key_point_flag=-1;
%     for i=1:size(crucial_judge_point,1)
%         if abs(crucial_judge_point(i,1)-In_point(:,1))<0.01 || abs(crucial_judge_point(i,2)-In_point(:,2))<0.01...
%             || abs(crucial_judge_point(i,1)-Out_point(:,1))<0.01 || abs(crucial_judge_point(i,2)-Out_point(:,2))<0.01
%             continue;
%         else
%             key_point_flag=i;
%             remain_point=[crucial_judge_point(i,:),0];
%             break;
%         end
%     end
%     if key_point_flag==-1 %都不是的情况下检查边的中点
%         %todo:还要加上一个选择哪条边上的中点
%         for i=1:size(mid_judge_point,1)
%             if abs(mid_judge_point(i,1)-In_point(:,1))<0.01 || abs(mid_judge_point(i,2)-In_point(:,2))<0.01 ||...
%                     abs(mid_judge_point(i,1)-Out_point(:,1))<0.01 || abs(mid_judge_point(i,2)-Out_point(:,2))<0.01
%                 remain_point=[mid_judge_point(2,:),1];
%                 break;
%             else
%                 remain_point=[mid_judge_point(1,:),1];
%             end
%         end 
%     end
%     gsp_point=[(remain_point(1,1)+(xmax+xmin)/2)/2,(remain_point(1,2)+(ymax+ymin)/2)/2,(In_theta+Out_theta)/2];


    gsp_point=[(xmax+xmin)/2,(ymax+ymin)/2,(In_theta+Out_theta)/2];


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
%     if((xmax>params_.scenario.xmax)||(xmin<params_.scenario.xmin)||(ymax>params_.scenario.ymax)||(ymin<params_.scenario.ymin))
%         return;
%     end
    if((xmax>params_.scenario.xmax-params_.vehicle.dual_disk_radius)||...
        (xmin<params_.scenario.xmin+params_.vehicle.dual_disk_radius)||...
        (ymax>params_.scenario.ymax-params_.vehicle.dual_disk_radius)||...
        (ymin<params_.scenario.ymin+params_.vehicle.dual_disk_radius))
        return;
    end
    
    
    
    point_nums=1000;
    vx = [linspace(xmin, xmax, point_nums), linspace(xmax, xmax, point_nums), linspace(xmax, xmin, point_nums), linspace(xmin, xmin, point_nums)];
    vy = [linspace(ymax, ymax, point_nums), linspace(ymax, ymin, point_nums), linspace(ymin, ymin, point_nums), linspace(ymin, ymax, point_nums)];
    
    
    
%     for ii = 1 : params_.obstacle.num_obs
%          if (any(inpolygon(vx, vy, params_.obs.old_dilated_obs{ii}.x,params_.obs.old_dilated_obs{ii}.y)))
%             return;
%          end
%     end
%     flag=1;
    ind_x=floor((vx-params_.scenario.xmin)/params_.hybrid_astar.resolution_dx)+1;
    ind_y=floor((vy-params_.scenario.ymin)/params_.hybrid_astar.resolution_dy)+1;
    if(any(params_.scenario.dilated_map(sub2ind(size(params_.scenario.dilated_map),ind_x,ind_y))))
        return;
    end
    flag=1;
end
function [x,y]=ModifyCenterPoint(narrow_x,narrow_y,theta)
    global params_
    x=narrow_x;
    y=narrow_y;
    for kk=2:length(narrow_x)-1
%         lb=getsafeposition(narrow_x(kk),narrow_y(kk),theta(kk));
        lb=getlength(narrow_x(kk),narrow_y(kk));
        point_num=5;
        center_x=narrow_x(kk)+(lb(4)-lb(2))/2;
        center_y=narrow_y(kk)+(lb(1)-lb(3))/2;
        posetheta=theta(kk);
        for i=0:1:point_num
           posex=narrow_x(kk)+i*(center_x-narrow_x(kk))/point_num;
           posey=narrow_y(kk)+i*(center_y-narrow_x(kk))/point_num;
           xr=posex+params_.vehicle.r2p*cos(posetheta);
           yr=posey+params_.vehicle.r2p*sin(posetheta);
           xf=posex+params_.vehicle.f2p*cos(posetheta);
           yf=posey+params_.vehicle.f2p*sin(posetheta);
           xx=[xr,xf];
           yy=[yr,yf];   
           ind_xx=floor((xx-params_.scenario.xmin)/params_.hybrid_astar.resolution_dx)+1;
           ind_yy=floor((yy-params_.scenario.ymin)/params_.hybrid_astar.resolution_dy)+1;
           if(~any(params_.scenario.dilated_map(sub2ind(size(params_.scenario.dilated_map),ind_xx,ind_yy))))
               x(kk)=posex;
               y(kk)=posey;               
               break;
           end
        end
    end
end
function lb=getsafeposition(xc,yc,theta)
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
            if(ispositionValid(xc,yc,test,ind,der_s,theta))
                test(ind)=test(ind)+der_s;
                lb=test;
            else
                is_completed(ind)=1;
            end
        end
    end
end
function flag=ispositionValid(x,y,trail,ind,der_s,theta)
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
%     if((xmax>params_.scenario.xmax)||(xmin<params_.scenario.xmin)||(ymax>params_.scenario.ymax)||(ymin<params_.scenario.ymin))
%         return;
%     end
    if((xmax>params_.scenario.xmax-params_.vehicle.dual_disk_radius)||...
        (xmin<params_.scenario.xmin+params_.vehicle.dual_disk_radius)||...
        (ymax>params_.scenario.ymax-params_.vehicle.dual_disk_radius)||...
        (ymin<params_.scenario.ymin+params_.vehicle.dual_disk_radius))
        return;
    end
    

    
    
    point_nums=100;
    vx = [linspace(xmin, xmax, point_nums), linspace(xmax, xmax, point_nums), linspace(xmax, xmin, point_nums), linspace(xmin, xmin, point_nums)];
    vy = [linspace(ymax, ymax, point_nums), linspace(ymax, ymin, point_nums), linspace(ymin, ymin, point_nums), linspace(ymin, ymax, point_nums)];
    
    o_theta=theta*ones(1,size(vx,2));
    xr=vx+params_.vehicle.r2p*cos(o_theta);
    yr=vy+params_.vehicle.r2p*sin(o_theta);
    xf=vx+params_.vehicle.f2p*cos(o_theta);
    yf=vy+params_.vehicle.f2p*sin(o_theta);
    xx=[xr,xf];
    yy=[yr,yf];   
    ind_xx=floor((xx-params_.scenario.xmin)/params_.hybrid_astar.resolution_dx)+1;
    ind_yy=floor((yy-params_.scenario.ymin)/params_.hybrid_astar.resolution_dy)+1;
    if(any(params_.scenario.dilated_map(sub2ind(size(params_.scenario.dilated_map),ind_xx,ind_yy))))
        return;
    end
    
%     for ii = 1 : params_.obstacle.num_obs
%          if (any(inpolygon(vx, vy, params_.obs.old_dilated_obs{ii}.x,params_.obs.old_dilated_obs{ii}.y)))
%             return;
%          end
%     end
%     flag=1;
    ind_x=floor((vx-params_.scenario.xmin)/params_.hybrid_astar.resolution_dx)+1;
    ind_y=floor((vy-params_.scenario.ymin)/params_.hybrid_astar.resolution_dy)+1;
    if(any(params_.scenario.dilated_map(sub2ind(size(params_.scenario.dilated_map),ind_x,ind_y))))
        return;
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
            break;
        end
        ccw_theta=i;
    end
    for j=0:-pi/24:-pi/2 
        if(~IsSpinTrajValid(xf,yf,thetaf+j))
            break;
        end
        cw_theta=j;
    end
    params_.hybrid_astar.spin_theta=[ccw_theta,cw_theta]; 
    DrawTrajFootprints(params_.task.xf,params_.task.yf,params_.task.thetaf+params_.hybrid_astar.spin_theta(1,1));
    DrawTrajFootprints(params_.task.xf,params_.task.yf,params_.task.thetaf+params_.hybrid_astar.spin_theta(1,2));
    
    xf=params_.task.xf;
    yf=params_.task.yf;
    thetaf=params_.task.thetaf;
    
    task_point=[];
    spin_theta=params_.hybrid_astar.spin_theta;%这个角度要在GetMapInformation中得到,这个角度要限制在[-pi/2,pi/2];
    spin_CCW=linspace(0,spin_theta(:,1),5);
    spin_CW=linspace(0,spin_theta(:,2),5);
    expansion_theta=[spin_CCW,spin_CW(1,2:end)];
    for ii=1:size(expansion_theta,2)
        task_point=[task_point;xf,yf,RegulateAngle(expansion_theta(:,ii)+thetaf)];
    end
    params_.task.task_point=task_point;
    
    
end
function is_collision_free=IsSpinTrajValid(x,y,theta)
    is_collision_free=0;
    global params_
    xr=x+params_.vehicle.r2p*cos(theta);
    yr=y+params_.vehicle.r2p*sin(theta);
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