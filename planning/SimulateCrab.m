function path_seg_crab=SimulateCrab(cur_node,v,phy)
    global params_
    nfe=40;
    x=zeros(1,nfe);
    y=zeros(1,nfe);
    theta=zeros(1,nfe);
    pattern=2*ones(1,nfe);
    
    x(1)=cur_node.x;
    y(1)=cur_node.y;
    theta(1)=cur_node.theta;
    dt=params_.hybrid_astar.simulation_step/(nfe-1);
    
    for ii=2:nfe
        theta(ii)=theta(1);
        theta_temp=theta(1)+phy;
        x(ii)=x(ii-1)+dt*cos(theta_temp)*v;
        y(ii)=y(ii-1)+dt*sin(theta_temp)*v;
    end    
    ind=round(linspace(1,40,10));
    path_seg_crab.x=x(ind);
    path_seg_crab.y=y(ind);
    path_seg_crab.theta=theta(ind);
    path_seg_crab.pattern=pattern(ind);
end