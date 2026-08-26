function GenerateGifForLimo()
    global params_
    [x,y,theta,~,~,phy,~]=ResampleCollocationPoints(params_.limo.x,params_.limo.y,params_.limo.theta,params_.limo.v,params_.limo.a,params_.limo.phy,params_.limo.w,params_.limo.terminal_time);

    DrawParkingScenario();
    name_str=['Limo_',num2str(params_.user.case_id)];
    name_str_gif=[pwd,'\DemoResults\',name_str,'.gif'];
    [I,map]=rgb2ind(frame2im(getframe(gcf)),256);
    imwrite(I,map,name_str_gif,'gif','Loopcount',inf,'DelayTime',0.05);

    timer=0;
    for ii=1:length(x)
        timer=params_.utility.traj_dt_for_resample*(ii-1);
        handle0=text(params_.scenario.xmax-18,params_.scenario.ymax-1,['Wall-clock time = ',num2str(timer,' %2.2f'),'sec.'],'FontSize',16,'FontWeight','bold');
        V=CreateVehiclePolygon(x(ii),y(ii),theta(ii),2);
        handle1=fill(V.x,V.y,params_.utility.ego_vehicle_rgb,'facealpha',0.15);
        [h1,h2,h3,h4,h5,h6]=DrawWheels(x(ii),y(ii),theta(ii),phy(ii));
        handle2=plot(x(1:ii),y(1:ii),'Color',[63,72,204]./255,'LineWidth',2);

        [I,map]=rgb2ind(frame2im(getframe(gcf)),256);
        imwrite(I,map,name_str_gif,'gif','WriteMode','append','DelayTime',0.01);

        if(ii~=length(x))
            delete(handle0);
            delete(handle1);
            delete(h1);
            delete(h2);
            delete(h3);
            delete(h4);
            delete(h5);
            delete(h6);
            delete(handle2);
        end
    end
end