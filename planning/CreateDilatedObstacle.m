function CreateDilatedObstacle()
    global params_
    for ii=1:size(params_.obstacle.obs,2)
        obs_x=params_.obstacle.obs{ii}.x;
        obs_y=params_.obstacle.obs{ii}.y;
%         r=ceil(params_.vehicle.dual_disk_radius);
        r=params_.vehicle.dual_disk_radius;
        before=polyshape(obs_x,obs_y);
        after=polybuffer(before,r,'JointType','miter','MiterLimit',2);
        dilated.x=[after.Vertices(:,1)',after.Vertices(1,1)];
        dilated.y=[after.Vertices(:,2)',after.Vertices(1,2)];
        params_.obs.old_dilated_obs{end+1}=dilated;
        params_.obs.old_height_value=[params_.obs.old_height_value,0.3];
%         plot(dilated.x,dilated.y,'LineWidth',2); % 原多边形区域
    end
end