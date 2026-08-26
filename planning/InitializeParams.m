function InitializeParams()
    global params_
    LoadCase();

    params_.utility.colorpool = [237,28,36; 0,162,232; 255,127,39; 218,112,214; 255,192,203; 123,104,238;0,0,255;0,0,139;119,136,153;30,144,255;70,130,180;0,191,255;0,139,139;255,102,0;0,250,154;127,255,0;154,205,50;255,215,0;205,133,63;128,0,0;0,255,255;240,128,128;255,0,0;105,105,105;169,169,169;192,192,192;0,0,0] ./ 255;
    params_.utility.ego_vehicle_rgb = [0.00, 0.45, 0.74];
    params_.utility.traj_dt_for_resample = 0.1;

    params_.scenario.xmin = -20;
    params_.scenario.xmax = 20;
    params_.scenario.ymin = -20;
    params_.scenario.ymax = 20;
%     %just for CaseNo_101
%     params_.scenario.xmin = 0;
%     params_.scenario.xmax = 30;
%     params_.scenario.ymin = 0;
%     params_.scenario.ymax = 30;

    params_.vehicle.lw = 2.8; % wheelbase
    params_.vehicle.lf = 0.96; % front hang length
    params_.vehicle.lr = 0.929; % rear hang length
    params_.vehicle.lb = 1.942; % width

    params_.vehicle.length = params_.vehicle.lw + params_.vehicle.lf + params_.vehicle.lr;
    params_.vehicle.hypotenuse_length = hypot(params_.vehicle.length, params_.vehicle.lb);
    params_.vehicle.dual_disk_radius = hypot(0.25 * params_.vehicle.length, 0.5 * params_.vehicle.lb); % Dual disk radius
    params_.vehicle.r2p = 0.25 * params_.vehicle.length - params_.vehicle.lr;
    params_.vehicle.f2p = 0.75 * params_.vehicle.length - params_.vehicle.lr;
    params_.vehicle.FWISr2p = 0.25* params_.vehicle.length;
    params_.vehicle.FWISf2p = 0.25 * params_.vehicle.length;
    params_.vehicle.vmax = 2.5;
    params_.vehicle.amax = 1;
    params_.vehicle.phymax = 0.75;
    params_.vehicle.wmax = 0.5;
    params_.vehicle.kappa_max = tan(params_.vehicle.phymax) / params_.vehicle.lw;
    params_.vehicle.turning_radius_min = abs(1.0 / params_.vehicle.kappa_max);
    params_.vehicle.threshold_s = (params_.vehicle.vmax^2) / params_.vehicle.amax; % Is it correct?

    params_.hybrid_astar.num_nodes_x = 125;
    params_.hybrid_astar.num_nodes_y = 125;
    params_.hybrid_astar.resolution_dx = (params_.scenario.xmax - params_.scenario.xmin) / params_.hybrid_astar.num_nodes_x;
    params_.hybrid_astar.resolution_dy = (params_.scenario.ymax - params_.scenario.ymin) / params_.hybrid_astar.num_nodes_y;
    params_.hybrid_astar.num_nodes_theta = 30;
    params_.hybrid_astar.resolution_dtheta = 2 * pi / params_.hybrid_astar.num_nodes_theta;
    params_.hybrid_astar.num_sampled_phy_in_expansion = 7; % Should be set as an odd number
    params_.hybrid_astar.num_sampled_phy_in_crab=2;
    params_.hybrid_astar.num_sampled_phy_in_spin=8;
    params_.hybrid_astar.penalty_for_backward = 1;
    params_.hybrid_astar.penalty_for_direction_change = 1;
    params_.hybrid_astar.penalty_for_steering_change = 0.2;
    params_.hybrid_astar.multiplier_H = 1.5;
    params_.hybrid_astar.multiplier_H_for_2dim_A_star = 3.0;
    params_.hybrid_astar.simulation_step =1.5;
    params_.hybrid_astar.max_search_iter = 500;
    params_.hybrid_astar.threshold_to_trigger_rs = 5;
    params_.hybrid_astar.FindSpaceder_s=0.25;
    params_.hybrid_astar.FindSpacemaxs=5;
    
    params_.hybrid_astar.ackerman_to_normal=0.2;
    params_.hybrid_astar.crab_to_normal=0.2;
    params_.hybrid_astar.spin_to_normal=0.2;
    
    params_.hybrid_astar.normal_to_acherman=0.4;
    params_.hybrid_astar.normal_to_crab=0.6;
    params_.hybrid_astar.normal_to_spin=0.6;

    params_.opti.nfe =100;
    params_.opti.stc.ds_for_init_adjustment = 0.01;
    params_.opti.stc.ds = 0.1;
    params_.opti.der_s=0.5;
    params_.opti.stc.smax = 5;
    params_.opti.collision_probability_threshold=0.1;
%     params_.opti.acceptance_tolerance = 0.000001;
    params_.opti.acceptance_tolerance = 0.006;
    params_.opti.cost_function_external_penalty_weight = 100000;
    params_.opti.cost_a = 0.025;
    params_.opti.cost_w = 0.0051;
    params_.opti.cost_phy = 0.00083983;
    params_.opti.max_iter = 5;
    params_.corridor_adaptive=0.3;



    CreateCostmaps();
    WriteBasicParameterFile();
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
basic_elem = strel('disk', 0 + ceil(params_.vehicle.dual_disk_radius /length_unit));
params_.scenario.dilated_map = imdilate(costmap, basic_elem);
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
function LoadCase()
    global params_

    load([pwd,'\ParkingBenchmarks\CaseNo_',num2str(params_.user.case_id),'.mat']);
    params_.task.x0=x0;
    params_.task.y0=y0;
    params_.task.theta0=theta0;
    params_.task.xf=xtf;
    params_.task.yf=ytf;
    params_.task.thetaf=thetatf;
    params_.obstacle.num_obs=length(obstacles);
    params_.obstacle.obs=obstacles;
    WriteBoundaryValues();

    params_.user.demo.enable_gif_hybrid_a_star=1;
    params_.user.demo.enable_gif_stc=0;
    params_.user.demo.enable_plot_intermediate_optimum=0;
    params_.user.demo.enable_gif_liom=0;
    params_.user.demo.enable_final_result_tiff_and_figure=1;
    params_.user.demo.video_play_speed=6;
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
