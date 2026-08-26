function ConstructSafeTravelCorridors(x,y,filename)
    global params_
    box_edge_length=zeros(size(x,2),4);
    for ii=1:size(x,2)
        [xc,yc]=SpinTrial(x(ii),y(ii));
        lb=GetAabbLength(xc,yc);
        box_edge_length(ii,:)=[xc-lb(2),xc+lb(4),yc-lb(3),yc+lb(1)];
    end
    WriteStcConstraints(box_edge_length,filename);
    if(all(filename=='fron'))
        params_.opti.stc.front_stc=box_edge_length;
    elseif(all(filename=='rear'))
        params_.opti.stc.rear_stc=box_edge_length;
    end
end

function[xc,yc]=SpinTrial(xc,yc)
    if(IsPointValidInDilatedMap(xc,yc))
        return;
    end
    global params_
    unit_length=params_.hybrid_astar.resolution_dx*0.5;
    ii=0;
    while(1)
        ii=ii+1;
        angle_type=mod(ii,8);
        radius=1+(ii-angle_type)/8;
        angle=angle_type*pi/4;
        x_nudge=xc+cos(angle)*radius*unit_length;
        y_nudge=yc+sin(angle)*radius*unit_length;
        if(IsPointValidInDilatedMap(x_nudge,y_nudge))
            xc=x_nudge;
            yc=y_nudge;
            return;
        end
    end
end

function is_valid=IsPointValidInDilatedMap(x,y)
    is_valid=0;
    global params_
    ind_x=floor((x-params_.scenario.xmin)/params_.hybrid_astar.resolution_dx)+1;
    ind_y=floor((y-params_.scenario.ymin)/params_.hybrid_astar.resolution_dy)+1;
    if((ind_x<1)||(ind_x>params_.hybrid_astar.num_nodes_x)||(ind_y<1)||(ind_y>params_.hybrid_astar.num_nodes_y))
        return;
    end
    if(params_.scenario.dilated_map(sub2ind(size(params_.scenario.dilated_map),ind_x,ind_y)))
        return;
    end
    is_valid=1;
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

    ax=xc-lb(2);ay=yc+lb(1);
    bx=xc+lb(4);by=yc+lb(1);
    cx=xc+lb(4);cy=yc-lb(3);
    dx=xc-lb(2);dy=yc-lb(3);

    global params_
    ds=params_.hybrid_astar.resolution_dx*0.5;
    switch ind
    case 1
        xmax=bx;xmin=ax;ymin=ay;ymax=yc+test(1);
    case 2
        xmax=ax;xmin=xc-test(2);ymin=dy;ymax=ay;
    case 3
        xmax=cx;xmin=dx;ymin=yc-test(3);ymax=cy;
    case 4
        xmax=xc+test(4);xmin=cx;ymin=cy;ymax=by;
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

function WriteStcConstraints(box_edge_length,filename)
    delete(['STC_',filename]);
    fid=fopen(['STC_',filename],'w');
    for ii=1:length(box_edge_length)
        for jj=1:4
            fprintf(fid,'%g %g %.6f \r\n',ii,jj,box_edge_length(ii,jj));
        end
    end
    fclose(fid);
end