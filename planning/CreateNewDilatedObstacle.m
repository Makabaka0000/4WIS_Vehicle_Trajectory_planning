function CreateNewDilatedObstacle()
    global params_
    obs_x=params_.obs.new{end}.x;
    obs_y=params_.obs.new{end}.y;
    r=params_.vehicle.dual_disk_radius;
    before=polyshape(obs_x,obs_y);
    after=polybuffer(before,r,'JointType','miter','MiterLimit',2);
    dilated.x=[after.Vertices(:,1)',after.Vertices(1,1)];
    dilated.y=[after.Vertices(:,2)',after.Vertices(1,2)];
    params_.obs.new_dilated_obs{end+1}=dilated;
%     plot(dilated.x,dilated.y,'LineWidth',2); % 原多边形区域
%     plot(obs_x,obs_y,'LineWidth',2);
end