function addnewobstacle()
    global params_
    xmin = params_.scenario.xmin;
    ymin = params_.scenario.ymin;
    resolution_x = params_.hybrid_astar.resolution_dx;
    resolution_y = params_.hybrid_astar.resolution_dy;
%     costmap = params_.scenario.original_map;
    costmap = params_.scenario.dilated_map;
    obs_vx =  params_.obs.new_dilated_obs{end}.x;
    obs_vy =  params_.obs.new_dilated_obs{end}.y;
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
%             vx = [linspace(x0, x1, 5), linspace(x1, x1, 5), linspace(x1, x0, 5), linspace(x0, x0, 5)];
%             vy = [linspace(y1, y1, 5), linspace(y1, y0, 5), linspace(y0, y0, 5), linspace(y0, y1, 5)];
%             if (any(inpolygon(vx, vy,params_.obs.new_dilated_obs{end}.x, params_.obs.new_dilated_obs{end}.y)))
%                 costmap(jj, kk) = 1;
%             end
            vx = [x0,x0,x1,x1,x0];
            vy = [y0,y1,y1,y0,y0];
                %只有在轮廓中才算
            [in,~]=inpolygon(vx, vy, params_.obs.new_dilated_obs{end}.x, params_.obs.new_dilated_obs{end}.y);
            if (sum(in)==size(in,2))
                costmap(jj, kk) = 1;
            end
        end
    end
%     params_.scenario.original_map = costmap;
      params_.scenario.dilated_map=costmap;
%     length_unit = 0.5 * (resolution_x + resolution_y);
%     basic_elem = strel('disk', 0 + ceil(params_.vehicle.dual_disk_radius / length_unit));
%     params_.scenario.dilated_map = imdilate(costmap, basic_elem);
end
function [ind1, ind2] = ConvertXyToId(x, y)
    global params_
    ind1 = ceil((x - params_.scenario.xmin) / params_.hybrid_astar.resolution_dx) + 1;
    ind2 = ceil((y - params_.scenario.ymin) / params_.hybrid_astar.resolution_dy) + 1;
    if ((ind1 <= params_.hybrid_astar.num_nodes_x)&&(ind1 >= 1)&&(ind2 <= params_.hybrid_astar.num_nodes_y)&&(ind2 >= 1))
        return;
    end
    if (ind1 > params_.hybrid_astar.num_nodes_x)
        ind1 = params_.hybrid_astar.num_nodes_x;
    elseif (ind1 < 1)
        ind1 = 1;
    end
    if (ind2 > params_.hybrid_astar.num_nodes_y)
        ind2 = params_.hybrid_astar.num_nodes_y;
    elseif (ind2 < 1)
        ind2 = 1;
    end
end