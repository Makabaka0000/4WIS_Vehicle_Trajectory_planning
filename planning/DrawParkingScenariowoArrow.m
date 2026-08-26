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