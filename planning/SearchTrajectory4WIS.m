function SearchTrajectory4WIS(IsGearShift,gsp_point)
    global params_
    if IsGearShift==1
        [x,y,theta,pattern]=SearchForwardHybridAstar([params_.task.x0,params_.task.y0,params_.task.theta0],[params_.task.xf,params_.task.yf,params_.task.thetaf]);
    elseif IsGearShift==2
        [x,y,theta,pattern]=SearchBackwardHybridAStar([params_.task.x0,params_.task.y0,params_.task.theta0],[params_.task.xf,params_.task.yf,params_.task.thetaf]);
    elseif IsGearShift==3
        [x_f,y_f,theta_f,pattern_f]=SearchGSPForwardHybridAstar([params_.task.x0,params_.task.y0,params_.task.theta0],gsp_point,1);
%         for i=1:size(x_f,2)
%             Arrow([x_f(1,i),y_f(1,i)],[x_f(1,i)+cos(theta_f(1,i)),y_f(1,i)+sin(theta_f(1,i))],'Length',16,'BaseAngle',90,'TipAngle',16,'Width',2);
%         end
        [x_b,y_b,theta_b,pattern_b]=SearchGSPBackwardHybridAstar(gsp_point,[params_.task.xf,params_.task.yf,params_.task.thetaf],2);
%         for i=1:size(x_b,2)
%             Arrow([x_b(1,i),y_b(1,i)],[x_b(1,i)+cos(theta_b(1,i)),y_b(1,i)+sin(theta_b(1,i))],'Length',16,'BaseAngle',90,'TipAngle',16,'Width',2);
%         end
        x=[x_f,x_b];
        y=[y_f,y_b];
        theta=[theta_f,theta_b];
%         for i=1:size(x,2)
%             Arrow([x(1,i),y(1,i)],[x(1,i)+cos(theta(1,i)),y(1,i)+sin(theta(1,i))],'Length',16,'BaseAngle',90,'TipAngle',16,'Width',2);
%         end
        pattern=[pattern_f,pattern_b];
    else
        [x_b,y_b,theta_b,pattern_b]=SearchGSPBackwardHybridAstar([params_.task.x0,params_.task.y0,params_.task.theta0],gsp_point,1);
        [x_f,y_f,theta_f,pattern_f]=SearchGSPForwardHybridAstar(gsp_point,[params_.task.xf,params_.task.yf,params_.task.thetaf],2);
        x=[x_b,x_f];
        y=[y_b,y_f];
        theta=[theta_b,theta_f];
        pattern=[pattern_b,pattern_f];
    end
%     [params_.ha_result.x,params_.ha_result.y,params_.ha_result.theta,...1:
%     params_.ha_result.v,params_.ha_result.a,params_.ha_result.phy,...
%     params_.ha_result.w,params_.ha_result.terminal_time]=ResamplePath(x,y,theta);
    %分段速度优化
    [params_.ha_result.x,params_.ha_result.y,params_.ha_result.theta,...
    params_.ha_result.v,params_.ha_result.a,params_.ha_result.phy,...
    params_.ha_result.w,params_.ha_result.terminal_time,params_.ha_result.Index]=ReSample(x,y,theta,pattern);
end
function [ha_x,ha_y,ha_theta,ha_v,ha_a,ha_phy,ha_w,ha_terminal_time,Index]=ReSample(x,y,theta,pattern)
    ha_x=[];ha_y=[];ha_theta=[];ha_v=[];ha_a=[];ha_phy=[];ha_w=[];ha_terminal_time=0;
    Index=[];
    Index_seg=getpatternindex(pattern);
    for i=1:size(Index_seg,1)
        if Index_seg(i,3)==1
            continue
        end
        [x_seg,y_seg,theta_seg,v_seg,a_seg,phy_seg,w_seg,terminal_time_seg]=ResamplePath4WIS(Index_seg(i,:),x,y,theta);
        ha_x=[ha_x,x_seg];
        ha_y=[ha_y,y_seg];
        ha_theta=[ha_theta,theta_seg];
        ha_v=[ha_v,v_seg];
        ha_a=[ha_a,a_seg];
        ha_phy=[ha_phy,phy_seg];
        ha_w=[ha_w,w_seg];
        ha_terminal_time=ha_terminal_time+terminal_time_seg;
        Index=[Index;size(ha_x,2)-size(x_seg,2)+1,size(ha_x,2),Index_seg(i,3),terminal_time_seg];
    end
end
function Index_seg=getpatternindex(pattern)
    Index_start=1;
    Index_end=1;
    Index_seg=[];
    while Index_end <= size(pattern,2)
        if pattern(Index_end)==pattern(Index_start)
            if Index_end== size(pattern,2)
                Index_seg=[Index_seg;Index_start,Index_end,pattern(Index_start)];
            end
        else
            Index_seg=[Index_seg;Index_start,Index_end-1,pattern(Index_start)];
            Index_start=Index_end;
        end
        Index_end=Index_end+1;
    end
end