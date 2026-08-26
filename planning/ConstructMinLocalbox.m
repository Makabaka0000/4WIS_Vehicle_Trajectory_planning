function ConstructMinLocalbox(x,y,filename)
    global params_
    %采样点之间间隔不是很远，可认为两点之间的曲线用直线连接
    boundary_point=[x(1),y(1)];
    for i=1:size(x,2)
        boundary_point=[boundary_point;x(1,i),y(1,i)];
    end
    boundary_point=[boundary_point;x(end),y(end)];
    % boundary_point=[];
    % for i=1:size(x,2)
    %     boundary_point=[boundary_point;x(1,i),y(1,i)];
    % end
    %计算每个矩形框每个方向的拓展边长，找到这三个点的最大最小x值，最大最小y值
    box_edge_length=zeros(params_.opti.nfe,4);
    for ii=2:size(boundary_point,1)-1
        limit_length=CaculateLimit(boundary_point,ii);
        lb=GetAabbLength(x(ii-1),y(ii-1),limit_length);
        box_edge_length(ii-1,:)=[x(ii-1)-lb(4),x(ii-1)+lb(2),y(ii-1)-lb(3),y(ii-1)+lb(1)];
    end
    if(all(filename=='fron'))
        params_.stc.min_front=box_edge_length;
    elseif(all(filename=='rear'))
        params_.stc.min_rear=box_edge_length;
    end
% DrawParkingScenario();
% plot(x,y,'LineWidth',2);
% for i=1:size(box_edge_length,1)
% %     xmin=box_edge_length(i,1);
% %     xmax=box_edge_length(i,2);
% %     ymin=box_edge_length(i,3);
% %     ymax=box_edge_length(i,4);
%     xmin=box_edge_length(i,1);
%     xmax=box_edge_length(i,3);
%     ymin=box_edge_length(i,4);
%     ymax=box_edge_length(i,6);
%     plot([xmin,xmin,xmax,xmax,xmin],[ymin,ymax,ymax,ymin,ymin],'LineWidth',1,'color','green');
% end
end
function limit_length=CaculateLimit(boundary_point,index)
    point_before=boundary_point(index-1,:);
    point_cur=boundary_point(index,:);
    point_after=boundary_point(index+1,:);
    x1=point_before(1,1)-point_cur(1,1);
    y1=point_before(1,2)-point_cur(1,2);
    x2=point_after(1,1)-point_cur(1,1);
    y2=point_after(1,2)-point_cur(1,2);
    if (x1>0 && x2<0)
        right=abs(x1);
        left=abs(x2);
    elseif(x1>0 && x2>0)
        right=max(abs(x1),abs(x2));
        left=min(abs(x1),abs(x2));
    elseif(x1<0 && x2<0)
        left=max(abs(x1),abs(x2));
        right=min(abs(x1),abs(x2));
    else
        left=abs(x1);
        right=abs(x2);
    end
    if (x1==0 || x2==0)
        right=left;
    end
    if (y1>0 && y2<0)
        up=abs(y1);
        down=abs(y2);
    elseif(y1>0 && y2>0)
        up=max(abs(y1),abs(y2));
        down=min(abs(y1),abs(y2));
    elseif(y1<0 && y2<0)
        down=max(abs(y1),abs(y2));
        up=min(abs(y1),abs(y2));
    else
        down=abs(y1);
        up=abs(y2);
    end
    if (y1==0 || y2==0)
        up=down;
    end
    limit_length=[up,right,down,left];
    for j=1:size(limit_length,2)
        if limit_length(1,j)<0.1
            limit_length(1,j)=0.3;
        end
    end
end
function lb=GetAabbLength(xc,yc,limit_length)
    lb=zeros(1,4);
    is_completed=zeros(1,4);
    stc_dc=0.05;
    while(sum(is_completed)<4)
        for ind=1:4
            if(is_completed(ind))
                continue;
            end
            test=lb;
            if(test(ind)+stc_dc>limit_length(ind))
                is_completed(ind)=1;
                continue;
            end
            test(ind)=test(ind)+stc_dc;
            if(IsCurrentExpansionValid(xc,yc,test,lb,ind))
                lb=test;
            else
                is_completed(ind)=1;
            end
        end
    end
end
function is_valid=IsCurrentExpansionValid(xc,yc,test,lb,ind)
    is_valid=0;
    x_min=xc-lb(4);
    x_max=xc+lb(2);
    y_min=yc-lb(3);
    y_max=yc+lb(1);

    global params_
    ds=params_.hybrid_astar.resolution_dx*0.5;
    switch ind
    case 1
        xmax=x_max;xmin=x_min;ymin=y_max;ymax=y_max+test(1);
    case 2
        xmax=x_max+test(2);xmin=x_max;ymin=y_min;ymax=y_max;
    case 3
        xmax=x_max;xmin=x_min;ymin=y_min-test(3);ymax=y_min;
    case 4
        xmax=x_min;xmin=x_min-test(4);ymin=y_min;ymax=y_max;
    otherwise
        return;
    end

    if((xmax>params_.scenario.xmax-params_.vehicle.dual_disk_radius)||...
        (xmin<params_.scenario.xmin+params_.vehicle.dual_disk_radius)||...
        (ymax>params_.scenario.ymax-params_.vehicle.dual_disk_radius)||...
        (ymin<params_.scenario.ymin+params_.vehicle.dual_disk_radius))
        return;
    end

    xx=[];
    yy=[];
    nx=ceil((xmax-xmin)/ds)+1;
    ny=ceil((ymax-ymin)/ds)+1;
    for x=linspace(xmin,xmax,nx)
        for y=linspace(ymin,ymax,ny)
            xx=[xx,x];
            yy=[yy,y];
        end
    end
    ind_x=floor((xx-params_.scenario.xmin)/params_.hybrid_astar.resolution_dx)+1;
    ind_y=floor((yy-params_.scenario.ymin)/params_.hybrid_astar.resolution_dy)+1;
    if(any(params_.scenario.dilated_map(sub2ind(size(params_.scenario.dilated_map),ind_x,ind_y))))
        return;
    end
    is_valid=1;
end
