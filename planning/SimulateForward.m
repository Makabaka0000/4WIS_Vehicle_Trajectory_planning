function path_seg=SimulateForward(cur_node,v,phy)
    global params_
    nfe=40;
    x=zeros(1,nfe);
    y=zeros(1,nfe);
    theta=zeros(1,nfe);
    pattern=0*ones(1,nfe);
    
    x(1)=cur_node.x;
    y(1)=cur_node.y;
    theta(1)=cur_node.theta;
    dt=params_.hybrid_astar.simulation_step/(nfe-1);
    
    for ii=2:nfe
        theta(ii)=theta(ii-1)+2*dt*tan(phy)*v/params_.vehicle.lw;
        x(ii)=x(ii-1)+dt*cos(theta(ii-1))*v;
        y(ii)=y(ii-1)+dt*sin(theta(ii-1))*v;
    end

    ind=round(linspace(1,40,10));
    path_seg.x=x(ind);
    path_seg.y=y(ind);
    path_seg.theta=theta(ind);
    path_seg.pattern=pattern(ind);
end