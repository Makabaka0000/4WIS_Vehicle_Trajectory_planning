function ConstructMaxLocalbox(x,y,filename)
    global params_
    box_edge_length=zeros(params_.opti.nfe,4);
    for ii=1:params_.opti.nfe
        lb=GetAabbLength(x(ii),y(ii));
        box_edge_length(ii,:)=[x(ii)-lb(4),x(ii)+lb(2),y(ii)-lb(3),y(ii)+lb(1)];
    end
    if(all(filename=='fron'))
        params_.opti.stc.front_stc=box_edge_length;
    elseif(all(filename=='rear'))
        params_.opti.stc.rear_stc=box_edge_length;
    end
%     DrawParkingScenario();
%     plot(x,y,'LineWidth',2);
%     for i=1:size(box_edge_length,1)
%         xmin=box_edge_length(i,1);
%         xmax=box_edge_length(i,2);
%         ymin=box_edge_length(i,3);
%         ymax=box_edge_length(i,4);
%         plot([xmin,xmin,xmax,xmax,xmin],[ymin,ymax,ymax,ymin,ymin],'LineWidth',1,'color','green');
%     end
end
function lb=GetAabbLength(xc,yc)
    global params_
    lb=zeros(1,4);
    is_completed=zeros(1,4);
    while(sum(is_completed)<4)
        for ind=1:4
            if(is_completed(ind))
                continue;
            end
            test=lb;
            if(test(ind)+params_.opti.stc.ds>params_.opti.stc.smax)
                is_completed(ind)=1;
                continue;
            end
            test(ind)=test(ind)+params_.opti.stc.ds;
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



