close all; clc; clear global params_; clear all;
global params_
for ii =66:66

    params_.user.case_id = ii;
    MyInitPar();
        %     [x,y,theta]=SearchAStarPath([params_.task.x0,params_.task.y0,params_.task.theta0],[params_.task.xf,params_.task.yf,params_.task.thetaf]);
     [traj_x,traj_y,~]=SearchAStarPath([params_.task.x0,params_.task.y0,params_.task.theta0],[params_.task.xf,params_.task.yf,params_.task.thetaf]);
     DrawParkingScenario();
     plot(traj_x,traj_y);
    [guided_points,IsGearShift]=SetGuidedPoint(traj_x,traj_y);
    %segment Hybrid A*
    %todo:在guided point搜索中要reverse尝试能不能连接searchforward或者searchbackward的点
    %如果有的话把其连接起来，避免不合理的搜索
    [x,y,theta,ready_flag]=SegmentCrossHybridAstar(guided_points,IsGearShift);
        [params_.ha_result.x,params_.ha_result.y,params_.ha_result.theta,...
        params_.ha_result.v,params_.ha_result.a,params_.ha_result.phy,...
        params_.ha_result.w,params_.ha_result.terminal_time]=ResamplePath(x,y,theta);
%     OptimizeTrajectory4WIS();
end
function DrawParkingScenariowoArrow()
    global params_
    close all;
    figure(params_.user.case_id);
    set(0,'DefaultLineLineWidth',1);
    hold on;
    box on;
    grid minor;
    axis equal;
    axis([params_.scenario.xmin,params_.scenario.xmax,params_.scenario.ymin,params_.scenario.ymax]);
    set(gcf,'outerposition',get(0,'screensize'));
    for jj=1:params_.obstacle.num_obs
        V=params_.obstacle.obs{jj};
        fill(V.x,V.y,[0,0,0]);
    end
    xlabel('x / m','FontSize',17,'FontName','Arial Narrow','FontWeight','Bold');
    ylabel('y / m','FontSize',17,'FontName','Arial Narrow','FontWeight','Bold');
end