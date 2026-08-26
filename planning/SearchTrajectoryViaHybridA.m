function ready_flag=SearchTrajectoryViaHybridA()
    global params_
    [x,y,theta,ready_flag]=SearchHybridAStarPath();
    [params_.ha_result.x,params_.ha_result.y,params_.ha_result.theta,...
    params_.ha_result.v,params_.ha_result.a,params_.ha_result.phy,...
    params_.ha_result.w,params_.ha_result.terminal_time]=ResamplePath(x,y,theta);
    if(~ready_flag)
        x=params_.ha_result.x;
        y=params_.ha_result.y;
        theta=params_.ha_result.theta;
        v=params_.ha_result.v;
        a=params_.ha_result.a;
        phy=params_.ha_result.phy;
        w=params_.ha_result.w;
        terminal_time=params_.ha_result.terminal_time;
        save(strcat('E:\MATLAB 2019a\bin\mycor\dilemma\hybrid\initguess',num2str(params_.user.case_id)),'x','y','theta','v','a','phy','w','terminal_time');    
    end
end

function[x,y,theta,ready_flag]=SearchHybridAStarPath()
    clear global openlist_;
    clear global grid_space_;
    global params_ openlist_ grid_space_

    if(params_.user.demo.enable_gif_hybrid_a_star)
        DrawParkingScenario();
        name_str=['HA_',num2str(params_.user.case_id)];
        name_str_gif=[pwd,'\DemoResults\',name_str,'.gif'];
        [I,map]=rgb2ind(frame2im(getframe(gcf)),256);
        imwrite(I,map,name_str_gif,'gif','Loopcount',inf,'DelayTime',0.05);
    end

    expansion_pattern=SpecifySamplePattern();
    grid_space_=cell(params_.hybrid_astar.num_nodes_x,params_.hybrid_astar.num_nodes_y,params_.hybrid_astar.num_nodes_theta);
    goal_node.x=params_.task.xf;
    goal_node.y=params_.task.yf;
    goal_node.theta=RegulateAngle(params_.task.thetaf);
    goal_ind=Convert3DimConfigToIndex(goal_node);

    init_node.x=params_.task.x0;
    init_node.y=params_.task.y0;
    init_node.theta=RegulateAngle(params_.task.theta0);
    init_node.h=CalculateH(init_node);
    init_node.g=0;
    init_node.f=init_node.g+init_node.h;
    init_node.is_in_openlist=1;
    init_node.is_in_closedlist=0;
    init_node.id=Convert3DimConfigToIndex(init_node);
    init_node.parent_id=[-999,-999,-999];
    init_node.via_phy=0;
    init_node.via_v=0;
    init_node.traj_from_parent_to_cur.x=[];
    init_node.traj_from_parent_to_cur.y=[];
    init_node.traj_from_parent_to_cur.theta=[];
    openlist_=[init_node.f,init_node.id];
    RegisterNodeInGridSpace(init_node);

    counter_to_trigger_rs_curve=0;
    is_ha_search_complete=0;
    params_.hybrid_astar.is_a_search_feasible=1;
    is_ha_search_completed_by_rs=0;
    current_best_cost=init_node.f;
    current_best_node=init_node;
    ha_iter=0;
    while((~isempty(openlist_))&&(ha_iter<=params_.hybrid_astar.max_search_iter)&&(~is_ha_search_complete))
        ha_iter=ha_iter+1;
        if(params_.user.demo.enable_gif_hybrid_a_star)
            plot_iter_handle=text(params_.scenario.xmax-5,params_.scenario.ymax-1,['Iter = ',num2str(ha_iter,'%4d')],'FontSize',16,'FontWeight','bold');
        end
        cur_node=ExtractBestNodeFromOpenlist();

        cur_node_normalized_f_value=cur_node.g+cur_node.h/params_.hybrid_astar.multiplier_H;
        if(cur_node_normalized_f_value<current_best_cost)
            current_best_cost=cur_node_normalized_f_value;
            current_best_node=cur_node;
        end

        for ii=1:(2*params_.hybrid_astar.num_sampled_phy_in_expansion)
            child_node.via_v=expansion_pattern(ii,1);
            child_node.via_phy=expansion_pattern(ii,2);
            path_seg=SimulateForward(cur_node,child_node.via_v,child_node.via_phy);
            child_node.x=path_seg.x(end);
            child_node.y=path_seg.y(end);
            child_node.theta=RegulateAngle(path_seg.theta(end));
            child_node.parent_id=cur_node.id;
            child_node.traj_from_parent_to_cur.x=path_seg.x;
            child_node.traj_from_parent_to_cur.y=path_seg.y;
            child_node.traj_from_parent_to_cur.theta=path_seg.theta;
            child_node.id=Convert3DimConfigToIndex(child_node);

            if((~isempty(grid_space_{child_node.id(1),child_node.id(2),child_node.id(3)}))&&(grid_space_{child_node.id(1),child_node.id(2),child_node.id(3)}.is_in_closedlist==1))
                continue;
            end

            child_g=cur_node.g+params_.hybrid_astar.simulation_step+params_.hybrid_astar.penalty_for_direction_change*abs(child_node.via_v-cur_node.via_v)+params_.hybrid_astar.penalty_for_steering_change*abs(child_node.via_phy-cur_node.via_phy);
            if(child_node.via_v<0)
                child_g=child_g+params_.hybrid_astar.penalty_for_backward;
            end


            if(~isempty(grid_space_{child_node.id(1),child_node.id(2),child_node.id(3)}))

                if(grid_space_{child_node.id(1),child_node.id(2),child_node.id(3)}.g>child_g+0.001)
                    child_node.h=grid_space_{child_node.id(1),child_node.id(2),child_node.id(3)}.h;
                    child_node.f=child_g+child_node.h;
                    child_node.g=child_g;
                    child_node.is_in_closedlist=0;
                    child_node.is_in_openlist=1;
                    grid_space_{child_node.id(1),child_node.id(2),child_node.id(3)}=child_node;
                    DeleteNodeIDFromOpenlist(child_node.id);
                    openlist_=[openlist_;child_node.f,child_node.id];
                end
                continue;
            end


            if(~Is3DTrajValid(child_node.traj_from_parent_to_cur.x,child_node.traj_from_parent_to_cur.y,child_node.traj_from_parent_to_cur.theta))
                child_node.is_in_closedlist=1;
                grid_space_{child_node.id(1),child_node.id(2),child_node.id(3)}=child_node;
                continue;
            end



            child_node.h=CalculateH(child_node);
            child_node.g=child_g;
            child_node.f=child_node.g+child_node.h;
            child_node.is_in_openlist=1;
            child_node.is_in_closedlist=0;
            openlist_=[openlist_;child_node.f,child_node.id];
            RegisterNodeInGridSpace(child_node);


            if(params_.user.demo.enable_gif_hybrid_a_star)
                plot(child_node.traj_from_parent_to_cur.x,child_node.traj_from_parent_to_cur.y,'Color',[34,177,76]./255,'LineWidth',0.5);
                plot(child_node.x,child_node.y,'k.');
            end


            counter_to_trigger_rs_curve=counter_to_trigger_rs_curve+1;
            if(counter_to_trigger_rs_curve>params_.hybrid_astar.threshold_to_trigger_rs)
                counter_to_trigger_rs_curve=0;
                rs_path_seg=GenerateRsPath(child_node);
                if(Is3DTrajValid(rs_path_seg.x,rs_path_seg.y,rs_path_seg.theta))
                    is_ha_search_complete=1;
                    is_ha_search_completed_by_rs=1;
                    current_best_node=child_node;
                    break;
                end
            end

            if(~any(child_node.id-goal_ind))
                is_ha_search_complete=1;
                current_best_node=child_node;
                break;
            end
        end
        if(params_.user.demo.enable_gif_hybrid_a_star)
            [I,map]=rgb2ind(frame2im(getframe(gcf)),256);
            imwrite(I,map,name_str_gif,'gif','WriteMode','append','DelayTime',0.05);
            delete(plot_iter_handle);
        end
    end
    [x,y,theta]=BacktrackPath(current_best_node);
    if(is_ha_search_completed_by_rs)
        x=[x,rs_path_seg.x];
        y=[y,rs_path_seg.y];
        theta=[theta,rs_path_seg.theta];
    end

    if(params_.user.demo.enable_gif_hybrid_a_star)
        plot(x,y,'Color',[1,.5,.5],'LineWidth',3);
        if(is_ha_search_completed_by_rs)
            plot(rs_path_seg.x,rs_path_seg.y,'Color',[.5,1,.5],'LineWidth',3);drawnow;
        end
        [I,map]=rgb2ind(frame2im(getframe(gcf)),256);
        for i=1:10
            imwrite(I,map,name_str_gif,'gif','WriteMode','append','DelayTime',0.05);
        end
    end
    ready_flag=is_ha_search_complete;
end

function expansion_pattern=SpecifySamplePattern()
    global params_
    phy_list=linspace(-params_.vehicle.phymax,params_.vehicle.phymax,params_.hybrid_astar.num_sampled_phy_in_expansion);
    expansion_pattern=[];
    for ii=1:params_.hybrid_astar.num_sampled_phy_in_expansion
        expansion_pattern=[expansion_pattern;[1,phy_list(ii);-1,phy_list(ii)]];
    end
end

function idx=Convert3DimConfigToIndex(node)
    global params_
    ind1=floor((node.x-params_.scenario.xmin)/params_.hybrid_astar.resolution_dx)+1;
    ind2=floor((node.y-params_.scenario.xmin)/params_.hybrid_astar.resolution_dx)+1;
    ind3=floor(node.theta/params_.hybrid_astar.resolution_dtheta)+1;
    if(ind1>params_.hybrid_astar.num_nodes_x)
        ind1=params_.hybrid_astar.num_nodes_x;
    elseif(ind1<1)
        ind1=1;
    end
    if(ind2>params_.hybrid_astar.num_nodes_y)
        ind2=params_.hybrid_astar.num_nodes_y;
    elseif(ind2<1)
        ind2=1;
    end
    if(ind3>params_.hybrid_astar.num_nodes_theta)
        ind3=params_.hybrid_astar.num_nodes_theta;
    elseif(ind3<1)
        ind3=1;
    end
    idx=[ind1,ind2,ind3];
end

function val=CalculateH(cur_node)
    global params_
    cur_x=cur_node.x;
    cur_y=cur_node.y;
    cur_theta=cur_node.theta;

    distance_nonholonomic_without_collision_avoidance_1=hypot(params_.task.xf-cur_x,params_.task.yf-cur_y);
    distance_nonholonomic_without_collision_avoidance_2=CalculateRsPathLength([cur_x,cur_y,cur_theta],[params_.task.xf,params_.task.yf,params_.task.thetaf]);
    distance_holonomic_with_collision_avoidance=CalculateAStarPathLength([cur_x,cur_y],[params_.task.xf,params_.task.yf]);

    val=params_.hybrid_astar.multiplier_H*max([distance_nonholonomic_without_collision_avoidance_1,distance_nonholonomic_without_collision_avoidance_2,distance_holonomic_with_collision_avoidance]);
end

function path_length=CalculateRsPathLength(start_config,end_config)
    global params_
    reedsConnObj=robotics.ReedsSheppConnection('MinTurningRadius',params_.vehicle.turning_radius_min);
    reedsConnObj.ReverseCost=params_.hybrid_astar.penalty_for_backward+1.0;
    [pathSegObj,~]=connect(reedsConnObj,start_config,end_config);
    path_length=pathSegObj{1}.Length;
end

function path_length=CalculateAStarPathLength(start_config,end_config)
    global params_
    begin_config=start_config(1:2);
    end_config=end_config(1:2);
    grid_space_2D_=cell(params_.hybrid_astar.num_nodes_x,params_.hybrid_astar.num_nodes_y);
    init_node=zeros(1,11);
    init_node(1:2)=begin_config;
    init_node(4)=0;
    init_node(5)=sum(abs(init_node(1:2)-end_config));
    init_node(3)=init_node(4)+params_.hybrid_astar.multiplier_H*init_node(5)+0.001*randn;
    init_node(6)=1;
    init_node(8:9)=Convert2DimConfigToIndex(begin_config);
    init_node(10:11)=[-999,-999];
    openlist_=init_node;
    goal_ind=Convert2DimConfigToIndex(end_config);
    grid_space_2D_{init_node(8),init_node(9)}=init_node;
    expansion_pattern=[-1,1;-1,0;-1,-1;0,1;0,-1;1,1;1,0;1,-1].*params_.hybrid_astar.resolution_dx;
    expansion_length=[1.414;1;1.414;1;1;1.414;1;1.414].*params_.hybrid_astar.resolution_dx;
    iter=0;

    while((~isempty(openlist_))&&(iter<=params_.hybrid_astar.num_nodes_x^2))
        iter=iter+1;


        cur_node_order=find(openlist_(:,3)==min(openlist_(:,3)));cur_node_order=cur_node_order(end);
        cur_node=openlist_(cur_node_order,:);
        cur_config=cur_node(1:2);
        cur_ind=cur_node(8:9);
        cur_g=cur_node(4);

        openlist_(cur_node_order,:)=[];
        grid_space_2D_{cur_ind(1),cur_ind(2)}(6)=0;
        grid_space_2D_{cur_ind(1),cur_ind(2)}(7)=1;
        for ii=1:8
            child_node_config=cur_config+expansion_pattern(ii,:);
            child_node_ind=Convert2DimConfigToIndex(child_node_config);
            child_g=cur_g+expansion_length(ii);
            child_h=sum(abs(child_node_config-end_config));
            child_f=child_g+params_.hybrid_astar.multiplier_H*child_h;
            child_node_prepare=[child_node_config,child_f,child_g,child_h,1,0,child_node_ind,cur_ind];

            if(~isempty(grid_space_2D_{child_node_ind(1),child_node_ind(2)}))

                if(grid_space_2D_{child_node_ind(1),child_node_ind(2)}(7)==1)
                    continue;
                end


                if(grid_space_2D_{child_node_ind(1),child_node_ind(2)}(4)>child_g+0.1)
                    child_node_order1=find(openlist_(:,8)==child_node_ind(1));
                    child_node_order2=find(openlist_(child_node_order1,9)==child_node_ind(2));
                    openlist_(child_node_order1(child_node_order2),:)=[];
                    grid_space_2D_{child_node_ind(1),child_node_ind(2)}=child_node_prepare;
                    openlist_=[openlist_;child_node_prepare];
                end
            else

                if(Is2DNodeValid(child_node_config,child_node_ind))


                    openlist_=[openlist_;child_node_prepare];
                    grid_space_2D_{child_node_ind(1),child_node_ind(2)}=child_node_prepare;
                    if(sum(abs(child_node_ind-goal_ind))==0)
                        path_length=child_g;
                        return;
                    end
                else
                    child_node_prepare(7)=1;
                    child_node_prepare(6)=0;
                    grid_space_2D_{child_node_ind(1),child_node_ind(2)}=child_node_prepare;
                end
            end
        end
    end
    path_length=sum(abs(begin_config-end_config));
end

function is_collision_free=Is2DNodeValid(child_node_config,child_node_ind)
    is_collision_free=0;
    global params_
    if(params_.scenario.dilated_map(child_node_ind(1),child_node_ind(2))==1)
        return;
    end
    if((child_node_config(1)>params_.scenario.xmax)||(child_node_config(1)<params_.scenario.xmin)||(child_node_config(2)>params_.scenario.ymax)||(child_node_config(2)<params_.scenario.ymin))
        return;
    end
    is_collision_free=1;
end

function idx=Convert2DimConfigToIndex(config)
    global params_
    ind1=ceil((config(1)-params_.scenario.xmin)/params_.hybrid_astar.resolution_dx)+1;
    ind2=ceil((config(2)-params_.scenario.ymin)/params_.hybrid_astar.resolution_dy)+1;
    idx=[ind1,ind2];
    if((ind1<=params_.hybrid_astar.num_nodes_x)&&(ind1>=1)&&(ind2<=params_.hybrid_astar.num_nodes_y)&&(ind2>=1))
        return;
    end
    if(ind1>params_.hybrid_astar.num_nodes_x)
        ind1=params_.hybrid_astar.num_nodes_x;
    elseif(ind1<1)
        ind1=1;
    end
    if(ind2>params_.hybrid_astar.num_nodes_y)
        ind2=params_.hybrid_astar.num_nodes_y;
    elseif(ind2<1)
        ind2=1;
    end
    idx=[ind1,ind2];
end

function RegisterNodeInGridSpace(node)
    global grid_space_
    grid_space_{node.id(1),node.id(2),node.id(3)}=node;
end

function cur_node=ExtractBestNodeFromOpenlist()
    global grid_space_ openlist_
    list=openlist_(:,1);
    id=find(list==min(list));
    id=id(end);

    cur_node_id=openlist_(id,2:4);
    cur_node=grid_space_{cur_node_id(1),cur_node_id(2),cur_node_id(3)};
    openlist_(id,:)=[];
    grid_space_{cur_node_id(1),cur_node_id(2),cur_node_id(3)}.is_in_closedlist=1;
    grid_space_{cur_node_id(1),cur_node_id(2),cur_node_id(3)}.is_in_openlist=0;
end

function path_seg=SimulateForward(cur_node,v,phy)
    global params_
    nfe=40;
    x=zeros(1,nfe);
    y=zeros(1,nfe);
    theta=zeros(1,nfe);

    x(1)=cur_node.x;
    y(1)=cur_node.y;
    theta(1)=cur_node.theta;
    dt=params_.hybrid_astar.simulation_step/(nfe-1);
    for ii=2:nfe
        theta(ii)=theta(ii-1)+dt*tan(phy)*v/params_.vehicle.lw;
        x(ii)=x(ii-1)+dt*cos(theta(ii-1))*v;
        y(ii)=y(ii-1)+dt*sin(theta(ii-1))*v;
    end

    ind=round(linspace(1,40,10));
    path_seg.x=x(ind);
    path_seg.y=y(ind);
    path_seg.theta=theta(ind);
end

function is_collision_free=Is3DTrajValid(x,y,theta)
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

function traj=GenerateRsPath(child_node)
    global params_
    reedsConnObj=robotics.ReedsSheppConnection('MinTurningRadius',params_.vehicle.turning_radius_min);
    reedsConnObj.ReverseCost=1.0+params_.hybrid_astar.penalty_for_backward;
    [pathSegObj,~]=connect(reedsConnObj,[child_node.x,child_node.y,child_node.theta],[params_.task.xf,params_.task.yf,params_.task.thetaf]);
    poses=interpolate(pathSegObj{1},[0:params_.hybrid_astar.resolution_dx:pathSegObj{1}.Length]);
    traj.x=poses(:,1)';
    traj.y=poses(:,2)';
    traj.theta=poses(:,3)';
end

function DeleteNodeIDFromOpenlist(id)
    global openlist_
    ind_1=find(openlist_(:,2)==id(1));
    ind_2=find(openlist_(:,3)==id(2));
    ind_3=find(openlist_(:,4)==id(3));
    ind_12=intersect(ind_1,ind_2);
    ind=intersect(ind_12,ind_3);
    if(length(ind)~=1)
        error '[DeleteNodeIDFromOpenlist] More than one ID in openlist.';
    end
    openlist_(ind,:)=[];
end

function[x,y,theta]=BacktrackPath(current_best_node)
    global grid_space_
    x=[current_best_node.x];
    y=[current_best_node.y];
    theta=[current_best_node.theta];
    cur_node=grid_space_{current_best_node.id(1),current_best_node.id(2),current_best_node.id(3)};
    while(1)
        x=[cur_node.traj_from_parent_to_cur.x,x];
        y=[cur_node.traj_from_parent_to_cur.y,y];
        theta=[cur_node.traj_from_parent_to_cur.theta,theta];
        next_id=cur_node.parent_id;
        if(next_id(1)==-999)
            break;
        end
        cur_node=grid_space_{next_id(1),next_id(2),next_id(3)};
    end
end