function  CrossHybridstar()
    global params_
    %set guided points
     [traj_x,traj_y,~]=SearchAStarPath([params_.task.x0,params_.task.y0,params_.task.theta0],[params_.task.xf,params_.task.yf,params_.task.thetaf]);

     DrawParkingScenario();
     plot(traj_x,traj_y);
    [guided_points,IsGearShift]=SetGuidedPoint(traj_x,traj_y);
    %segment Hybrid A*
    %todo:在guided point搜索中要reverse尝试能不能连接searchforward或者searchbackward的点
    %如果有的话把其连接起来，避免不合理的搜索
    [x,y,theta]=SegmentCrossHybridAstar(guided_points,IsGearShift);
    [params_.ha_result.x,params_.ha_result.y,params_.ha_result.theta,...
    params_.ha_result.v,params_.ha_result.a,params_.ha_result.phy,...
    params_.ha_result.w,params_.ha_result.terminal_time]=ResamplePath(x,y,theta);
    
    %gear shift points optimization
     
end