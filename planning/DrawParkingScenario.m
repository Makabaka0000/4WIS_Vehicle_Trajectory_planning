function DrawParkingScenario()
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
        fill(V.x,V.y,[0.5,0.5,0.5]);
    end
    
%     close all;
%     figure(params_.user.case_id);
%     set(0,'DefaultLineLineWidth',1);
%     hold on;
%     box on;
%     grid minor;
%     axis equal;
%     axis([params_.scenario.xmin,params_.scenario.xmax,params_.scenario.ymin,params_.scenario.ymax]);
%     set(gcf,'outerposition',get(0,'screensize'));
%     xlabel('x / m','FontSize',17,'FontName','Arial Narrow','FontWeight','Bold');
%     ylabel('y / m','FontSize',17,'FontName','Arial Narrow','FontWeight','Bold');
% 
%     xmin=params_.scenario.xmin;
%     ymin=params_.scenario.ymin;
%     resolution_x=params_.hybrid_astar.resolution_dx;
%     resolution_y=params_.hybrid_astar.resolution_dy;
%     for jj=1:params_.hybrid_astar.num_nodes_x
%         for kk=1:params_.hybrid_astar.num_nodes_y
%             if(params_.scenario.dilated_map(jj,kk)==1)
%                 x0=xmin+(jj-1)*resolution_x;
%                 y0=ymin+(kk-1)*resolution_y;
%                 x1=xmin+jj*resolution_x;
%                 y1=ymin+kk*resolution_y;
%                 vx=[linspace(x0,x1,2),linspace(x1,x1,2),linspace(x1,x0,2),linspace(x0,x0,2)];
%                 vy=[linspace(y1,y1,2),linspace(y1,y0,2),linspace(y0,y0,2),linspace(y0,y1,2)];
%                 fill(vx,vy,[.5,.5,.5],'EdgeColor',[0,0,0]);
%             end
%         end
%     end
    
    
    
    
    Arrow([params_.task.x0,params_.task.y0],[params_.task.x0+cos(params_.task.theta0),params_.task.y0+sin(params_.task.theta0)],'Length',16,'BaseAngle',90,'TipAngle',16,'Width',2);
    Arrow([params_.task.xf,params_.task.yf],[params_.task.xf+cos(params_.task.thetaf),params_.task.yf+sin(params_.task.thetaf)],'Length',16,'BaseAngle',90,'TipAngle',16,'Width',2);
    xlabel('x / m','FontSize',17,'FontName','Arial Narrow','FontWeight','Bold');
    ylabel('y / m','FontSize',17,'FontName','Arial Narrow','FontWeight','Bold');
end