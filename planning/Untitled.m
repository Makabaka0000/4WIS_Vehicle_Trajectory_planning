
hold on;
box on;
grid minor;
axis equal;
axis([params_.scenario.xmin,params_.scenario.xmax,params_.scenario.ymin,params_.scenario.ymax]);
set(gcf,'outerposition',get(0,'screensize'));
    for jj=1:params_.obstacle.num_obs
        V=params_.obstacle.obs{jj};
        fill(V.x,V.y,[0.5,0.5,0.5]);
    end
DrawTrajprints(params_.limo.x,params_.limo.y,params_.limo.theta);
plot(params_.limo.x,params_.limo.y,'Color','r','LineWidth',2);
Arrow([params_.task.x0,params_.task.y0],[params_.task.x0+cos(params_.task.theta0),params_.task.y0+sin(params_.task.theta0)],'Length',16,'BaseAngle',90,'TipAngle',16,'Width',2);
Arrow([params_.task.xf,params_.task.yf],[params_.task.xf+cos(params_.task.thetaf),params_.task.yf+sin(params_.task.thetaf)],'Length',16,'BaseAngle',90,'TipAngle',16,'Width',2);


function DrawTrajprints(x,y,theta)
    global params_
    for ii=1:length(x)
        V=CreateVehiclePolygon(x(ii),y(ii),theta(ii),2);
%         plot(V.x, V.y, 'Color',params_.utility.ego_vehicle_rgb);
        plot(V.x, V.y, 'Color',[1,1,1]);
    end
end