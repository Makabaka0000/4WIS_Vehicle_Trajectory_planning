function[x,y,theta,pattern]=SearchGSPForwardHybridAstar(start_config,end_config,order)
    clear global openlist_;
    clear global grid_space_;
    global params_ openlist_ grid_space_

    if(params_.user.demo.enable_gif_hybrid_a_star)
        DrawParkingScenario();
        DrawTrajFootprints(start_config(1,1),start_config(1,2),start_config(1,3));
        DrawTrajFootprints(end_config(1,1),end_config(1,2),end_config(1,3));
        Arrow([start_config(1,1),start_config(1,2)],[start_config(1,1)+cos(start_config(1,3)),start_config(1,2)+sin(start_config(1,3))],'Length',16,'BaseAngle',90,'TipAngle',16,'Width',2);
        Arrow([end_config(1,1),end_config(1,2)],[end_config(1,1)+cos(end_config(1,3)),end_config(1,2)+sin(end_config(1,3))],'Length',16,'BaseAngle',90,'TipAngle',16,'Width',2);
        name_str=['HA_',num2str(params_.user.case_id)];
        name_str_gif=[pwd,'\DemoResults\',name_str,'.gif'];
        [I,map]=rgb2ind(frame2im(getframe(gcf)),256);
        imwrite(I,map,name_str_gif,'gif','Loopcount',inf,'DelayTime',0.05);
    end

    [expansion_pattern,expansion_crab,expansion_spin]=SpecifySample();
    
    grid_space_=cell(params_.hybrid_astar.num_nodes_x,params_.hybrid_astar.num_nodes_y,params_.hybrid_astar.num_nodes_theta);
    goal_node.x=end_config(1,1);
    goal_node.y=end_config(1,2);
    goal_node.theta=RegulateAngle(end_config(1,3));
    goal_ind=Convert3DimConfigToIndex(goal_node);
    
    CalculateAStarPathLength([goal_node.x,goal_node.y]);%核查一下是否计算正确
    
    init_node.x=start_config(1,1);
    init_node.y=start_config(1,2);
    init_node.theta=RegulateAngle(start_config(1,3));
    init_node.id=Convert3DimConfigToIndex(init_node);
    init_node.h=CalculateH(init_node,end_config);
    init_node.g=0;
    init_node.f=init_node.g+init_node.h;
    init_node.is_in_openlist=1;
    init_node.is_in_closedlist=0;
    init_node.parent_id=[-999,-999,-999];
    init_node.via_phy=0;
    init_node.via_v=0;
    init_node.traj_from_parent_to_cur.x=[];
    init_node.traj_from_parent_to_cur.y=[];
    init_node.traj_from_parent_to_cur.theta=[];
    init_node.traj_from_parent_to_cur.pattern=[];
    init_node.pattern=0;
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
        %拓展阿克曼转向运动的节点
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
            child_node.traj_from_parent_to_cur.pattern=path_seg.pattern;
            child_node.id=Convert3DimConfigToIndex(child_node);
            child_node.pattern=0;
            penalty_for_model_change=0;
            if(cur_node.pattern ~= child_node.pattern)
                if(cur_node.pattern+child_node.pattern==1)
                     penalty_for_model_change= (params_.hybrid_astar.spin_to_normal)*abs(cur_node.via_phy)+ params_.hybrid_astar.normal_to_acherman*(abs(child_node.via_phy));
                elseif(cur_node.pattern+child_node.pattern==2)
                     penalty_for_model_change= (params_.hybrid_astar.crab_to_normal)*abs(cur_node.via_phy)+ params_.hybrid_astar.normal_to_acherman*(abs(child_node.via_phy));
                end
            end
            
            if((~isempty(grid_space_{child_node.id(1),child_node.id(2),child_node.id(3)}))&&(grid_space_{child_node.id(1),child_node.id(2),child_node.id(3)}.is_in_closedlist==1))
                continue;
            end
            child_g=cur_node.g+params_.hybrid_astar.simulation_step+params_.hybrid_astar.penalty_for_direction_change*abs(child_node.via_v-cur_node.via_v)+...
            params_.hybrid_astar.penalty_for_steering_change*abs(child_node.via_phy-cur_node.via_phy)+penalty_for_model_change;
        
            if(child_node.via_v<0)
                child_g=child_g+10*params_.hybrid_astar.penalty_for_backward;
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

            child_node.h=CalculateH(child_node,end_config);
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
                if order==1
                    connect_path=GenerateConnectPath(child_node,end_config);
                elseif order==2
                    connect_path=GenerateConnectPath2(child_node);
                end
                [flag,connect_path_seg]=IsTrajValid(connect_path); 
                if(flag)
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
        if(is_ha_search_completed_by_rs)
            break;
        end
        %拓展原地转向运动的节点
%         for ii=1:size(expansion_spin,2)
% %             expansion_spin=linspace(0,2*pi,params_.hybrid_astar.num_sampled_phy_in_spin);
%             child_node_spin.via_v=0;
%             child_node_spin.via_phy=0;
%             child_node_spin.x=cur_node.x;
%             child_node_spin.y=cur_node.y;
%             child_node_spin.theta=RegulateAngle(cur_node.theta+expansion_spin(ii));
%             child_node_spin.parent_id=cur_node.id;
%             child_node_spin.traj_from_parent_to_cur.x=[];
%             child_node_spin.traj_from_parent_to_cur.y=[];
%             child_node_spin.traj_from_parent_to_cur.theta=[];
%             child_node_spin.id=Convert3DimConfigToIndex(child_node_spin);
%             child_node_spin.pattern=1;
%             penalty_for_model_change=0;
%             if(cur_node.pattern ~= child_node_spin.pattern)
%                 if(cur_node.pattern+child_node_spin.pattern==1)
%                      penalty_for_model_change= (params_.hybrid_astar.ackerman_to_normal)*abs(cur_node.via_phy)+ params_.hybrid_astar.normal_to_spin;
%                 elseif(cur_node.pattern+child_node_spin.pattern==3)
%                      penalty_for_model_change= (params_.hybrid_astar.spin_to_normal)*abs(cur_node.via_phy)+ params_.hybrid_astar.normal_to_spin;
%                 end
%             end
%             
%             if((~isempty(grid_space_{child_node_spin.id(1),child_node_spin.id(2),child_node_spin.id(3)}))&&(grid_space_{child_node_spin.id(1),child_node_spin.id(2),child_node_spin.id(3)}.is_in_closedlist==1))
%                 continue;
%             end
%             
%             %如果cur_node与child_node类型一样不允许来回切换角
%             child_spin_g = cur_node.g+penalty_for_model_change+params_.hybrid_astar.penalty_for_steering_change* abs(child_node_spin.theta);            
%             if(~isempty(grid_space_{child_node_spin.id(1),child_node_spin.id(2),child_node_spin.id(3)}))
% 
%                 if(grid_space_{child_node_spin.id(1),child_node_spin.id(2),child_node_spin.id(3)}.g>child_spin_g+0.001)
%                     child_node_spin.h=grid_space_{child_node_spin.id(1),child_node_spin.id(2),child_node_spin.id(3)}.h;
%                     child_node_spin.f=child_spin_g+child_node_spin.h;
%                     child_node_spin.g=child_spin_g;
%                     child_node_spin.is_in_closedlist=0;
%                     child_node_spin.is_in_openlist=1;
%                     grid_space_{child_node_spin.id(1),child_node_spin.id(2),child_node_spin.id(3)}=child_node_spin;
%                     DeleteNodeIDFromOpenlist(child_node_spin.id);
%                     openlist_=[openlist_;child_node_spin.f,child_node_spin.id];
%                 end
%                 continue;
%             end
% 
%             if(~IsSpinTrajValid(child_node_spin.x,child_node_spin.y,child_node_spin.theta+expansion_spin(ii),child_node_spin.theta))%此处是否发生碰撞要重新写个函数检查
%                 child_node_spin.is_in_closedlist=1;
%                 grid_space_{child_node_spin.id(1),child_node_spin.id(2),child_node_spin.id(3)}=child_node_spin;
%                 continue;
%             end
%             child_node_spin.h=CalculateH(child_node_spin,end_config);
%             child_node_spin.g=child_spin_g;
%             child_node_spin.f=child_node_spin.g+child_node_spin.h;
%             child_node_spin.is_in_openlist=1;
%             child_node_spin.is_in_closedlist=0;
%             openlist_=[openlist_;child_node_spin.f,child_node_spin.id];
%             RegisterNodeInGridSpace(child_node_spin);  
%             if(params_.user.demo.enable_gif_hybrid_a_star)
%                 Arrow([child_node_spin.x,child_node_spin.y],[child_node_spin.x+cos(child_node_spin.theta),child_node_spin.y+sin(child_node_spin.theta)],'Length',1.6,'BaseAngle',9,'TipAngle',1.6,'Width',0.2);
%             end 
%             counter_to_trigger_rs_curve=counter_to_trigger_rs_curve+1;
%             if(counter_to_trigger_rs_curve>params_.hybrid_astar.threshold_to_trigger_rs)
%                 counter_to_trigger_rs_curve=0;
%                 rs_path_seg=GenerateRsPath(child_node_spin,end_config);
%                 if(Is3DTrajValid(rs_path_seg.x,rs_path_seg.y,rs_path_seg.theta))
%                     is_ha_search_complete=1;
%                     is_ha_search_completed_by_rs=1;
%                     current_best_node=child_node_spin;
%                     break;
%                 end
%             end
%             if(~any(child_node_spin.id-goal_ind))
%                 is_ha_search_complete=1;
%                 current_best_node=child_node_spin;
%                 break;
%             end
%             
%         end
       %拓展斜向运动的节点
        for ii = 1 : size(expansion_crab,1)
            child_node_crab.via_v=expansion_crab(ii,1);
            child_node_crab.via_phy=expansion_crab(ii,2);
            path_seg_crab=SimulateCrab(cur_node,child_node_crab.via_v,child_node_crab.via_phy);
            child_node_crab.x=path_seg_crab.x(end);
            child_node_crab.y=path_seg_crab.y(end);
            child_node_crab.theta=RegulateAngle(path_seg_crab.theta(end));
            child_node_crab.parent_id=cur_node.id;
            child_node_crab.traj_from_parent_to_cur.x=path_seg_crab.x;
            child_node_crab.traj_from_parent_to_cur.y=path_seg_crab.y;
            child_node_crab.traj_from_parent_to_cur.theta=path_seg_crab.theta;
            child_node_crab.traj_from_parent_to_cur.pattern=path_seg_crab.pattern;
            child_node_crab.id=Convert3DimConfigToIndex(child_node_crab);
            child_node_crab.pattern = 2;
            penalty_for_model_change = 0;
            if(cur_node.pattern ~= child_node_crab.pattern)
                if(cur_node.pattern+child_node_crab.pattern==2)
                     penalty_for_model_change= (params_.hybrid_astar.ackerman_to_normal)*abs(cur_node.via_phy)+ params_.hybrid_astar.normal_to_crab;
                elseif(cur_node.pattern+child_node_crab.pattern==3)
                     penalty_for_model_change= params_.hybrid_astar.spin_to_normal+ params_.hybrid_astar.normal_to_crab;
                end
            end
            
            if((~isempty(grid_space_{child_node_crab.id(1),child_node_crab.id(2),child_node_crab.id(3)}))&&(grid_space_{child_node_crab.id(1),child_node_crab.id(2),child_node_crab.id(3)}.is_in_closedlist==1))
                continue;
            end
           
            child_crab_g = cur_node.g + params_.hybrid_astar.simulation_step + params_.hybrid_astar.penalty_for_direction_change*abs(child_node_crab.via_v-cur_node.via_v) +... 
            penalty_for_model_change + params_.hybrid_astar.penalty_for_steering_change*abs(child_node_crab.via_phy-cur_node.via_phy);
            if(child_node_crab.via_v<0)
                child_crab_g=child_crab_g+10*params_.hybrid_astar.penalty_for_backward;
            end            
            
            if(~isempty(grid_space_{child_node_crab.id(1),child_node_crab.id(2),child_node_crab.id(3)}))

                if(grid_space_{child_node_crab.id(1),child_node_crab.id(2),child_node_crab.id(3)}.g>child_crab_g+0.001)
                    child_node_crab.h=grid_space_{child_node_crab.id(1),child_node_crab.id(2),child_node_crab.id(3)}.h;
                    child_node_crab.f=child_crab_g+child_node_crab.h;
                    child_node_crab.g=child_crab_g;
                    child_node_crab.is_in_closedlist=0;
                    child_node_crab.is_in_openlist=1;
                    grid_space_{child_node_crab.id(1),child_node_crab.id(2),child_node_crab.id(3)}=child_node_crab;
                    DeleteNodeIDFromOpenlist(child_node_crab.id);
                    openlist_=[openlist_;child_node_crab.f,child_node_crab.id];
                end
                continue;
            end
            if(~Is3DTrajValid(child_node_crab.traj_from_parent_to_cur.x,child_node_crab.traj_from_parent_to_cur.y,child_node_crab.traj_from_parent_to_cur.theta))
                child_node_crab.is_in_closedlist=1;
                grid_space_{child_node_crab.id(1),child_node_crab.id(2),child_node_crab.id(3)}=child_node_crab;
                continue;
            end
            child_node_crab.h=CalculateH(child_node_crab,end_config);
            child_node_crab.g=child_crab_g;
            child_node_crab.f=child_node_crab.g+child_node_crab.h;
            child_node_crab.is_in_openlist=1;
            child_node_crab.is_in_closedlist=0;
            openlist_=[openlist_;child_node_crab.f,child_node_crab.id];
            RegisterNodeInGridSpace(child_node_crab);  
            if(params_.user.demo.enable_gif_hybrid_a_star)
                plot(child_node_crab.traj_from_parent_to_cur.x,child_node_crab.traj_from_parent_to_cur.y,'Color','b','LineWidth',0.5);
                plot(child_node_crab.x,child_node_crab.y,'k.');
            end
            counter_to_trigger_rs_curve=counter_to_trigger_rs_curve+1;
            if(counter_to_trigger_rs_curve>params_.hybrid_astar.threshold_to_trigger_rs)
                counter_to_trigger_rs_curve=0;
                if order==1
                    connect_path=GenerateConnectPath(child_node_crab,end_config);
                elseif order==2
                    connect_path=GenerateConnectPath2(child_node_crab);
                end
                [flag,connect_path_seg]=IsTrajValid(connect_path); 
                if(flag)
                    is_ha_search_complete=1;
                    is_ha_search_completed_by_rs=1;
                    current_best_node=child_node_crab;
                    break;
                end
            end
            if(~any(child_node_crab.id-goal_ind))
                is_ha_search_complete=1;
                current_best_node=child_node_crab;
                break;
            end   
        end      
        if(params_.user.demo.enable_gif_hybrid_a_star)
            [I,map]=rgb2ind(frame2im(getframe(gcf)),256);
            imwrite(I,map,name_str_gif,'gif','WriteMode','append','DelayTime',0.05);
            delete(plot_iter_handle);
        end
    end
    [x,y,theta,pattern]=BacktrackPath(current_best_node);
    if(is_ha_search_completed_by_rs)
        x=[x,connect_path_seg.x];
        y=[y,connect_path_seg.y];
        theta=[theta,connect_path_seg.theta];
        pattern=[pattern,connect_path_seg.pattern];
    end

    if(params_.user.demo.enable_gif_hybrid_a_star)
        plot(x,y,'Color',[1,.5,.5],'LineWidth',3);
        if(is_ha_search_completed_by_rs)
            plot(connect_path_seg.x,connect_path_seg.y,'Color',[.5,1,.5],'LineWidth',3);drawnow;
        end
        [I,map]=rgb2ind(frame2im(getframe(gcf)),256);
        for i=1:10
            imwrite(I,map,name_str_gif,'gif','WriteMode','append','DelayTime',0.05);
        end
    end
end

function [expansion_pattern,expansion_crab,expansion_spin]=SpecifySample()
    global params_
    phy_list=linspace(-params_.vehicle.phymax,params_.vehicle.phymax,params_.hybrid_astar.num_sampled_phy_in_expansion);
    crab_list=linspace(-params_.vehicle.phymax,params_.vehicle.phymax,params_.hybrid_astar.num_sampled_phy_in_crab);
    expansion_pattern=[];
    expansion_crab=[];
    %在原地转向的时候，单次转向不能超过pi,
    re_expansion_spin=linspace(0,-pi,params_.hybrid_astar.num_sampled_phy_in_spin/2);
    expansion_spin=[linspace(0,pi,params_.hybrid_astar.num_sampled_phy_in_spin/2),re_expansion_spin(1,2:end)];
    for ii=1:params_.hybrid_astar.num_sampled_phy_in_expansion
        expansion_pattern=[expansion_pattern;[1,phy_list(ii);-1,phy_list(ii)]];
    end
    for ii=1:params_.hybrid_astar.num_sampled_phy_in_crab
        expansion_crab=[expansion_crab;[1,crab_list(ii);-1,crab_list(ii)]];
    end
%     xf=params_.task.xf;
%     yf=params_.task.yf;
%     thetaf=params_.task.thetaf;
%     task_point=[];
%     spin_theta=params_.hybrid_astar.spin_theta;%这个角度要在GetMapInformation中得到,这个角度要限制在[-pi/2,pi/2];
%     spin_CCW=linspace(0,spin_theta(:,1),5);
%     spin_CW=linspace(0,spin_theta(:,2),5);
%     expansion_theta=[spin_CCW,spin_CW(1,2:end)];
%     for ii=1:size(expansion_theta,2)
%         task_point=[task_point;xf,yf,RegulateAngle(expansion_theta(:,ii)+thetaf)];
%     end
%     params_.task.task_point=task_point;
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

function val=CalculateH(cur_node,end_config)
    global params_
    cur_x=cur_node.x;
    cur_y=cur_node.y;
    cur_theta=cur_node.theta;
    goal_xf=end_config(1,1);
    goal_yf=end_config(1,2);
    goal_theta=RegulateAngle(end_config(1,3));
    
    astarcost=params_.hybrid_astar.astarcost;
    distance_nonholonomic_without_collision_avoidance_1=hypot(goal_xf-cur_x,goal_yf-cur_y);
    distance_nonholonomic_without_collision_avoidance_2=CalculateRsPathLength([cur_x,cur_y,cur_theta],[goal_xf,goal_yf,goal_theta]);
    distance_holonomic_with_collision_avoidance=astarcost(cur_node.id(:,1),cur_node.id(:,2));

    val=params_.hybrid_astar.multiplier_H*max([distance_nonholonomic_without_collision_avoidance_1,distance_nonholonomic_without_collision_avoidance_2,distance_holonomic_with_collision_avoidance]);
end

function path_length=CalculateRsPathLength(start_config,end_config)
    global params_
    reedsConnObj=robotics.ReedsSheppConnection('MinTurningRadius',params_.vehicle.turning_radius_min);
    reedsConnObj.ReverseCost=params_.hybrid_astar.penalty_for_backward+1.0;
    [pathSegObj,~]=connect(reedsConnObj,start_config,end_config);
    path_length=pathSegObj{1}.Length;
end

function CalculateAStarPathLength(end_config)
    global params_
    begin_config=end_config(1:2);
    grid_space_2D_=cell(params_.hybrid_astar.num_nodes_x,params_.hybrid_astar.num_nodes_y);
    init_node=zeros(1,11);
    init_node(1:2)=begin_config;
    init_node(4)=0;
    init_node(5)=0;
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
    
    path_length=zeros(params_.hybrid_astar.num_nodes_x,params_.hybrid_astar.num_nodes_y);
    path_length(init_node(:,8),init_node(:,9))=init_node(:,3);

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
            child_h=0;
            child_f=child_g+params_.hybrid_astar.multiplier_H*child_h;
            child_node_prepare=[child_node_config,child_f,child_g,child_h,1,0,child_node_ind,cur_ind];

            if(~isempty(grid_space_2D_{child_node_ind(1),child_node_ind(2)}))

                if(grid_space_2D_{child_node_ind(1),child_node_ind(2)}(7)==1)
                    continue;
                end


                if(grid_space_2D_{child_node_ind(1),child_node_ind(2)}(4)>child_f+0.1)
                    child_node_order1=find(openlist_(:,8)==child_node_ind(1));
                    child_node_order2=find(openlist_(child_node_order1,9)==child_node_ind(2));
                    openlist_(child_node_order1(child_node_order2),:)=[];
                    grid_space_2D_{child_node_ind(1),child_node_ind(2)}=child_node_prepare;
                    openlist_=[openlist_;child_node_prepare];
                    path_length(child_node_ind(1),child_node_ind(2))=child_f;
                end
            else

                if(Is2DNodeValid(child_node_config,child_node_ind))
                    openlist_=[openlist_;child_node_prepare];
                    grid_space_2D_{child_node_ind(1),child_node_ind(2)}=child_node_prepare;
                    path_length(child_node_ind(1),child_node_ind(2))=child_f;
                else
                    child_node_prepare(7)=1;
                    child_node_prepare(6)=0;
                    grid_space_2D_{child_node_ind(1),child_node_ind(2)}=child_node_prepare;
                    path_length(child_node_ind(1),child_node_ind(2))=child_f;
                end
            end
        end
    end
    params_.hybrid_astar.astarcost=path_length;
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

function is_collision_free=IsSpinTrajValid(x,y,parent_theta,child_theta)
    is_collision_free=0;
    global params_
    %x,y坐标不会变化，但是圆盘的坐标会发生变化，插值theta,然后计算，所以输入进来的就是(x,y,转过的角度)
    theta=linspace(parent_theta,child_theta,10);
    xr=[];yr=[];xf=[];yf=[];
    for ii=1:size(theta,2)
        xr=[xr,x-params_.vehicle.FWISr2p*cos(theta)];
        yr=[yr,y-params_.vehicle.FWISr2p*sin(theta)];
        xf=[xf,x+params_.vehicle.FWISf2p*cos(theta)];
        yf=[yf,y+params_.vehicle.FWISf2p*sin(theta)];
    end
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
function traj=GenerateRsPath(child_node,end_config)
    global params_
    reedsConnObj=robotics.ReedsSheppConnection('MinTurningRadius',params_.vehicle.turning_radius_min);
    reedsConnObj.ReverseCost=1.0+params_.hybrid_astar.penalty_for_backward;
    [pathSegObj,~]=connect(reedsConnObj,[child_node.x,child_node.y,child_node.theta],[end_config(1,1),end_config(1,2),end_config(1,3)]);
    poses=interpolate(pathSegObj{1},[0:params_.hybrid_astar.resolution_dx:pathSegObj{1}.Length]);
    traj.x=poses(:,1)';
    traj.y=poses(:,2)';
    traj.theta=poses(:,3)';
end
function Traj=GenerateConnectPath(child_node,end_config)
    global params_
    Traj=[];
    
%     dubinsConnObj=robotics.DubinsConnection('MinTurningRadius',params_.vehicle.turning_radius_min/2);
%     [pathSegObj,~]=connect(dubinsConnObj,[child_node.x,child_node.y,child_node.theta],[end_config(1,1),end_config(1,2),end_config(1,3)]);

    reedsConnObj=robotics.ReedsSheppConnection('MinTurningRadius',params_.vehicle.turning_radius_min/2);
    reedsConnObj.ReverseCost=1.0+params_.hybrid_astar.penalty_for_backward;
    [pathSegObj,~]=connect(reedsConnObj,[child_node.x,child_node.y,child_node.theta],[end_config(1,1),end_config(1,2),end_config(1,3)]); 
    
    poses=interpolate(pathSegObj{1},[0:params_.hybrid_astar.resolution_dx:pathSegObj{1}.Length]);
    traj.x=poses(:,1)';
    traj.y=poses(:,2)';
    traj.theta=poses(:,3)';
    traj.pattern=0*ones(1,size(poses(:,1),1));
    Traj=[Traj;traj];
    clear traj;
    
    path_length=hypot(child_node.x-end_config(1,1),child_node.y-end_config(1,2));
    point_num=floor(path_length/params_.hybrid_astar.resolution_dx);
    posex=child_node.x;
    posey=child_node.y;
    posetheta=end_config(1,3);
    for k=1:point_num
       posex=[posex,child_node.x+(k+1)*(end_config(1,1)-child_node.x)/(point_num+1)];
       posey=[posey,child_node.y+(k+1)*(end_config(1,2)-child_node.y)/(point_num+1)];
       posetheta=[posetheta,end_config(1,3)];
    end
    traj.x=posex;
    traj.y=posey;
    traj.theta=posetheta;
    traj.pattern=2*ones(1,size(posex,2));
    Traj=[Traj;traj];
end
function Traj=GenerateConnectPath2(child_node)
    global params_
    task_point=params_.task.task_point;
    Traj=[];
    reedsConnObj=robotics.ReedsSheppConnection('MinTurningRadius',params_.vehicle.turning_radius_min/2);
    reedsConnObj.ReverseCost=1.0+params_.hybrid_astar.penalty_for_backward;

    
%     dubinsConnObj=robotics.DubinsConnection('MinTurningRadius',params_.vehicle.turning_radius_min/2);
    for i=1:size(task_point,1)
%         [pathSegObj,~]=connect(dubinsConnObj,[child_node.x,child_node.y,child_node.theta],[task_point(i,1),task_point(i,2),task_point(i,3)]);
        [pathSegObj,~]=connect(reedsConnObj,[child_node.x,child_node.y,child_node.theta],[task_point(i,1),task_point(i,2),task_point(i,3)]); 
        poses=interpolate(pathSegObj{1},[0:params_.hybrid_astar.resolution_dx:pathSegObj{1}.Length]);
        traj.x=poses(:,1)';
        traj.y=poses(:,2)';
        traj.theta=poses(:,3)';
        traj.pattern=0*ones(1,size(poses(:,1),1));
%         plot(traj.x,traj.y,'LineWidth',2);
%         hold on
        Traj=[Traj;traj];
        clear traj;
    end
    for i=1:size(task_point,1)
        path_length=hypot(child_node.x-task_point(i,1),child_node.y-task_point(i,2));
        point_num=floor(path_length/params_.hybrid_astar.resolution_dx);
        posex=child_node.x;
        posey=child_node.y;
        posetheta=task_point(1,3);
        for k=1:point_num
           posex=[posex,child_node.x+(k+1)*(task_point(i,1)-child_node.x)/(point_num+1)];
           posey=[posey,child_node.y+(k+1)*(task_point(i,2)-child_node.y)/(point_num+1)];
           posetheta=[posetheta,task_point(1,3)];
        end
        traj.x=posex;
        traj.y=posey;
        traj.theta=posetheta;
        traj.pattern=2*ones(1,size(posex,2));
%         plot(traj.x,traj.y,'LineWidth',2);
%         hold on
        Traj=[Traj;traj];
        clear traj;
    end    
end
function [is_collision_free,rs_seg]=IsTrajValid(Traj)
    global params_
    is_collision_free=0;
    rs_seg=[];
    for i=1:size(Traj,1)
        x=Traj(i,1).x;
        y=Traj(i,1).y;
        theta=Traj(i,1).theta;
%         plot(x,y,'LineWidth',2);
%         hold on
        xr=x+params_.vehicle.r2p*cos(theta);
        yr=y+params_.vehicle.r2p*sin(theta);
        xf=x+params_.vehicle.f2p*cos(theta);
        yf=y+params_.vehicle.f2p*sin(theta);
        xx=[xr,xf];
        yy=[yr,yf];
        if(any(xx>params_.scenario.xmax-params_.vehicle.dual_disk_radius*1.01))
            continue
        elseif(any(xx<params_.scenario.xmin+params_.vehicle.dual_disk_radius*1.01))
            continue
        elseif(any(yy>params_.scenario.ymax-params_.vehicle.dual_disk_radius*1.01))
            continue
        elseif(any(yy<params_.scenario.ymin+params_.vehicle.dual_disk_radius*1.01))
            continue
        end
        ind_x=floor((xx-params_.scenario.xmin)/params_.hybrid_astar.resolution_dx)+1;
        ind_y=floor((yy-params_.scenario.ymin)/params_.hybrid_astar.resolution_dy)+1;
        if(any(params_.scenario.dilated_map(sub2ind(size(params_.scenario.dilated_map),ind_x,ind_y))))
            continue
        end
        is_collision_free=1;
        rs_seg=Traj(i,1);
        break;
    end
end

function [flag,rs_seg]=IsRsTrajValid(Traj)
    global params_
    is_collision_free=zeros(1,size(Traj,1));
    is_valid=zeros(1,size(Traj,1));
    for i=1:size(Traj,1)
        x=Traj(i,1).x;
        y=Traj(i,1).y;
        theta=Traj(i,1).theta;
        xr=x-params_.vehicle.FWISr2p*cos(theta);
        yr=y-params_.vehicle.FWISr2p*sin(theta);
        xf=x+params_.vehicle.FWISf2p*cos(theta);
        yf=y+params_.vehicle.FWISf2p*sin(theta);
        xx=[xr,xf];
        yy=[yr,yf];
        if(any(xx>params_.scenario.xmax-params_.vehicle.dual_disk_radius*1.01))
            is_collision_free(1,i)=1;
            continue
        elseif(any(xx<params_.scenario.xmin+params_.vehicle.dual_disk_radius*1.01))
            is_collision_free(1,i)=1;
            continue
        elseif(any(yy>params_.scenario.ymax-params_.vehicle.dual_disk_radius*1.01))
            is_collision_free(1,i)=1;
            continue
        elseif(any(yy<params_.scenario.ymin+params_.vehicle.dual_disk_radius*1.01))
            is_collision_free(1,i)=1;
            continue
        end
        ind_x=floor((xx-params_.scenario.xmin)/params_.hybrid_astar.resolution_dx)+1;
        ind_y=floor((yy-params_.scenario.ymin)/params_.hybrid_astar.resolution_dy)+1;
        if(any(params_.scenario.dilated_map(sub2ind(size(params_.scenario.dilated_map),ind_x,ind_y))))
            is_collision_free(1,i)=1;
            continue
        end       
    end
    %find the index that is_collision_free(index)=0
    id=find(is_collision_free==min(is_collision_free));
    for j=id
        theta=Traj(j,1).theta;
        for k=1:size(theta,2)-1
            if(abs(theta(k+1)-theta(k))>pi/2)
                is_valid(1,j)=1;
                break;
            end
        end
    end
    res=is_collision_free+is_valid;
    traj=find(res==0);
    if isempty(traj)
        flag=0;
        rs_seg=[];
        return;
    else
        index=traj(1,1);
        flag=1;
        rs_seg=Traj(index,:);
    end
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
function[x,y,theta,pattern]=BacktrackPath(current_best_node)
    global grid_space_
    x=[current_best_node.x];
    y=[current_best_node.y];
    theta=[current_best_node.theta];
    pattern=[current_best_node.pattern];
    cur_node=grid_space_{current_best_node.id(1),current_best_node.id(2),current_best_node.id(3)};
    while(1)
        x=[cur_node.traj_from_parent_to_cur.x,x];
        y=[cur_node.traj_from_parent_to_cur.y,y];
        theta=[cur_node.traj_from_parent_to_cur.theta,theta];
        pattern=[cur_node.traj_from_parent_to_cur.pattern,pattern];
        next_id=cur_node.parent_id;
        if(next_id(1)==-999)
            break;
        end
        cur_node=grid_space_{next_id(1),next_id(2),next_id(3)};
    end
end