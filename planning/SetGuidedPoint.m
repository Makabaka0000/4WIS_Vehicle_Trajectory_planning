function [guided_points,IsGearShift]=SetGuidedPoint(x,y)
%     len=length(x);
%     [~,~,keypoint_index]=lineofsight(x,y);
%     reverse_x=flip(x);
%     reverse_y=flip(y);
%     [~,~,keypoint_reverse_index]=lineofsight(reverse_x,reverse_y);
%     partial_x=x(1,min(keypoint_index(2),len+1-keypoint_reverse_index(2)):max(keypoint_index(2),len+1-keypoint_reverse_index(2)));
%     partial_y=y(1,min(keypoint_index(2),len+1-keypoint_reverse_index(2)):max(keypoint_index(2),len+1-keypoint_reverse_index(2)));
%     [point_x,point_y,~]=lineofsight(partial_x,partial_y);
    global params_
    % 暂时先考虑从起点到终点的guided points,guided point距离不能太近
    x(1,1)=params_.task.x0;x(1,end)=params_.task.xf;
    y(1,1)=params_.task.y0;y(1,end)=params_.task.yf;
    origin_path.x=x;origin_path.y=y;
%     origin_path.theta_forward=caculatethetaforward(x,y);origin_path.theta_backward=caculatethetabackward(x,y);
%     length=0;
%     for i=1:size(x,2)-1
%         length=length+hypot(x(i+1)-x(i),y(i+1)-y(i));
%     end
    [~,~,keypoint_index]=lineofsight(x,y);
    [point_x,point_y,point_theta,IsGearShift]=selectdirection(keypoint_index,origin_path);
    guided_points.x=point_x;
    guided_points.y=point_y;
    guided_points.theta=point_theta;
    guided_points.num=length(point_x);
end
function [keypoint_x,keypoint_y,keypoint_index]=lineofsight(x,y)
    global params_
    keypoint_x=x(1);
    keypoint_y=y(1);
    keypoint_index=1;
%     obs_nums=params_.obstacle.num_obs;
%     obs=params_.obs.old_dilated_obs;
    obs_nums=params_.obstacle.num_obs;
    obs=params_.obstacle.obs;
    for i=2:length(x)
        lineseg=[keypoint_x(end),keypoint_y(end);x(i),y(i)];
        for k=1:obs_nums
            ob_area= polyshape(obs{k}.x,obs{k}.y);
            [in,~]=intersect(ob_area,lineseg);
            if(any(in))
                keypoint_x=[keypoint_x,x(i-1)];
                keypoint_y=[keypoint_y,y(i-1)];
                keypoint_index=[keypoint_index,i-1];
                break;
            end
        end
    end
    keypoint_x=[keypoint_x,x(end)];
    keypoint_y=[keypoint_y,y(end)];
    keypoint_index=[keypoint_index,length(x)];



%     global params_
%     keypoint_x=x(1);
%     keypoint_y=y(1);
%     keypoint_index=1;
%     for i=2:length(x)
%         path_length=hypot(keypoint_x(end)-x(i),keypoint_y(end)-y(i));
%         point_num=floor(path_length/params_.hybrid_astar.resolution_dx);
%         posex=keypoint_x(end);
%         posey=keypoint_y(end);
%         for k=1:point_num
%            posex=[posex,keypoint_x(end)+(k+1)*(x(i)-keypoint_x(end))/(point_num+1)];
%            posey=[posey,keypoint_y(end)+(k+1)*(y(i)-keypoint_y(end))/(point_num+1)];
%         end
%         ind_x=floor((posex-params_.scenario.xmin)/params_.hybrid_astar.resolution_dx)+1;
%         ind_y=floor((posey-params_.scenario.ymin)/params_.hybrid_astar.resolution_dy)+1;
%         if(any(params_.scenario.dilated_map(sub2ind(size(params_.scenario.dilated_map),ind_x,ind_y))))
%             keypoint_x=[keypoint_x,x(i-1)];
%             keypoint_y=[keypoint_y,y(i-1)];
%             keypoint_index=[keypoint_index,i-1];
%         end
%     end
%     keypoint_x=[keypoint_x,x(end)];
%     keypoint_y=[keypoint_y,y(end)];
%     keypoint_index=[keypoint_index,length(x)];
end
% function  theta_forward=caculatethetaforward(x,y)
%     global params_
%     theta=x;
%     theta(1)=params_.task.theta0;
%     theta(end)=params_.task.thetaf;
%     for ii=2:size(x,2)-1
%         theta(ii)=atan2(y(ii)-y(ii-1),x(ii)-x(ii-1));
%         while(theta(ii)>2*pi)
%             theta(ii)=theta(ii)-2*pi;
%         end
%         while(theta(ii)<0)
%             theta(ii)=theta(ii)+2*pi;
%         end
%     end
% %     for z=1:length(theta)
% %          Arrow([x(z),y(z)],[ x(z)+cos(theta(z)),y(z)+sin(theta(z))],'Length',16,'BaseAngle',90,'TipAngle',16,'Width',2);   
% %     end
%     theta_forward=theta;
% end
% function theta_b=caculatethetabackward(x,y)
%     global params_
%     x=flip(x);
%     y=flip(y);
%     theta=x;
%     theta(1)=params_.task.thetaf;
%     theta(end)=params_.task.theta0;
%     for ii=2:size(x,2)-1
%         theta(ii)=atan2(y(ii)-y(ii-1),x(ii)-x(ii-1));
%         while(theta(ii)>2*pi)
%             theta(ii)=theta(ii)-2*pi;
%         end
%         while(theta(ii)<-2*pi)
%             theta(ii)=theta(ii)+2*pi;
%         end
%     end
% 
% %     for z=1:length(theta)
% %          Arrow([x(z),y(z)],[ x(z)+cos(theta(z)),y(z)+sin(theta(z))],'Length',16,'BaseAngle',90,'TipAngle',16,'Width',2);   
% %     end
%     theta_b=flip(theta);
% end
% function is_valid=IspointValid(x,y,theta)
%     is_valid=0;
%     global params_
%     xr=x+params_.vehicle.r2p*cos(theta);
%     yr=y+params_.vehicle.r2p*sin(theta);
%     xf=x+params_.vehicle.f2p*cos(theta);
%     yf=y+params_.vehicle.f2p*sin(theta);
%     xx=[xr,xf];
%     yy=[yr,yf];
%     if(any(xx>params_.scenario.xmax-params_.vehicle.dual_disk_radius*1.01))
%         return;
%     elseif(any(xx<params_.scenario.xmin+params_.vehicle.dual_disk_radius*1.01))
%         return;
%     elseif(any(yy>params_.scenario.ymax-params_.vehicle.dual_disk_radius*1.01))
%         return;
%     elseif(any(yy<params_.scenario.ymin+params_.vehicle.dual_disk_radius*1.01))
%         return;
%     end
%     ind_x=floor((xx-params_.scenario.xmin)/params_.hybrid_astar.resolution_dx)+1;
%     ind_y=floor((yy-params_.scenario.ymin)/params_.hybrid_astar.resolution_dy)+1;
%     if(any(params_.scenario.dilated_map(sub2ind(size(params_.scenario.dilated_map),ind_x,ind_y))))
%         return;
%     end
%     is_valid=1;
% end