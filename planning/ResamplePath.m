function[x1,y1,theta1,v,a,phy,w,terminal_time]=ResamplePath(x,y,theta)
    global params_
    xx=x(1);
    yy=y(1);
    tt=theta(1);
    for ii=2:length(x)
        if((x(ii)==x(ii-1))&&(y(ii)==y(ii-1)))
            continue;
        end
        xx=[xx,x(ii)];
        yy=[yy,y(ii)];
        tt=[tt,theta(ii)];
    end
    x=xx;
    y=yy;
    theta=tt;

    if(size(x,2)==1)
        x1=ones(1,params_.opti.nfe).*x;
        y1=ones(1,params_.opti.nfe).*y;
        theta1=ones(1,params_.opti.nfe).*theta;
        v=zeros(1,params_.opti.nfe);
        a=zeros(1,params_.opti.nfe);
        phy=zeros(1,params_.opti.nfe);
        w=zeros(1,params_.opti.nfe);
        terminal_time=zeros(1,params_.opti.nfe);
        return;
    end

    theta(1)=params_.task.theta0;
    Nfe=length(theta);
    for ii=2:Nfe
        while(theta(ii)-theta(ii-1)>pi)
            theta(ii)=theta(ii)-2*pi;
        end
        while(theta(ii)-theta(ii-1)<-pi)
            theta(ii)=theta(ii)+2*pi;
        end
    end

    vdr=zeros(1,Nfe);
    for ii=2:(Nfe-1)
        addtion=(x(ii+1)-x(ii))*cos(theta(ii))+(y(ii+1)-y(ii))*sin(theta(ii));
        if(addtion>0)
            vdr(ii)=1;
        else
            vdr(ii)=-1;
        end
    end
    line_segment=cell(1,1);
    cur_index=1;
    counter=1;
    for ii=3:Nfe
        if(vdr(ii)~=vdr(ii-1))
            vec.x=x(1,cur_index:ii);
            vec.y=y(1,cur_index:ii);
            vec.theta=theta(1,cur_index:ii);
            vec.vdr=vdr(ii-1);
            line_segment{1,counter}=vec;
            counter=counter+1;
            cur_index=ii;
        end
    end

    x1=[];
    y1=[];
    theta1=[];
    v=[];
    a=[];
    terminal_time=0;
    for ii=1:(counter-1)
        [end_time,x0,y0,theta0,v0,a0]=CalculateTimeStamp(line_segment{1,ii},terminal_time);
        terminal_time=end_time;
        N=length(x0);
        x1=[x1,x0(1:(N-1))];
        y1=[y1,y0(1:(N-1))];
        theta1=[theta1,theta0(1:(N-1))];
        v=[v,v0(1:(N-1))];
        a=[a,a0(1:(N-1))];
    end
    x1=[x1,x0(end)];
    y1=[y1,y0(end)];
    theta1=[theta1,theta0(end)];
    v=[v,0];
    a=[a,0];

    Nfe=length(x1);
    phy=zeros(1,Nfe);
    w=zeros(1,Nfe);
    phy_max=params_.vehicle.phymax;
    w_max=params_.vehicle.wmax;
    dt=terminal_time/Nfe;


    for ii=2:(Nfe-1)
        phy(ii)=atan((theta1(ii+1)-theta1(ii))*params_.vehicle.lw/(dt*v(ii)));
        if(phy(ii)>phy_max)
            phy(ii)=phy_max;
        elseif(phy(ii)<-phy_max)
            phy(ii)=-phy_max;
        end
    end

    for ii=2:(Nfe-1)
        w(ii)=(phy(ii+1)-phy(ii))/dt;
        if(w(ii)>w_max)
            w(ii)=w_max;
        elseif(w(ii)<-w_max)
            w(ii)=-w_max;
        end
    end
    index=round(linspace(1,length(x1),params_.opti.nfe));
    x1=x1(index);
    y1=y1(index);
    theta1=theta1(index);
    v=v(index);
    a=a(index);
    phy=phy(index);
    w=w(index);

    theta1(1)=params_.task.theta0;
    for ii=2:length(theta1)
        while(theta1(ii)-theta1(ii-1)>pi)
            theta1(ii)=theta1(ii)-2*pi;
        end
        while(theta1(ii)-theta1(ii-1)<-pi)
            theta1(ii)=theta1(ii)+2*pi;
        end
    end
end

function[end_time,x0,y0,theta0,v,a]=CalculateTimeStamp(elem,begin_time)

    Nfe=length(elem.x);
    path_length=sum(hypot(elem.x(1,2:Nfe)-elem.x(1,1:(Nfe-1)),elem.y(1,2:Nfe)-elem.y(1,1:(Nfe-1))));

    [s,v,a,terminal_time]=SolveMinTimeOPtimalControlProblem(path_length);

    end_time=terminal_time+begin_time;

    [x0,y0,theta0]=InterpolateGrids(elem.x,elem.y,elem.theta);
    local_s=linspace(0,path_length,length(x0));

    ind=zeros(1,size(s,2));
    for ii=1:length(s)
        temp=abs(s(ii)-local_s);
        temp=find(temp==min(temp));
        ind(ii)=temp(end);
    end
    x0=x0(ind);
    y0=y0(ind);
    theta0=theta0(ind);
    if(elem.vdr<0)
        v=v.*-1;
        a=a.*-1;
    end
end

function[x_full,y_full,theta_full]=InterpolateGrids(x,y,theta)
    x_full=[];
    y_full=[];
    theta_full=[];
    for ii=2:length(x)
        Nsp=round(norm([x(ii)-x(ii-1),y(ii)-y(ii-1)])*200);
        temp=linspace(x(ii-1),x(ii),Nsp);
        x_full=[x_full,temp(1,1:(Nsp-1))];

        temp=linspace(y(ii-1),y(ii),Nsp);
        y_full=[y_full,temp(1,1:(Nsp-1))];

        temp=linspace(theta(ii-1),theta(ii),Nsp);
        theta_full=[theta_full,temp(1,1:(Nsp-1))];
    end
    x_full=[x_full,x(end)];
    y_full=[y_full,y(end)];
    theta_full=[theta_full,theta(end)];
end

function[s,v,a,terminal_time]=SolveMinTimeOPtimalControlProblem(path_length)
    global params_
    if(path_length<=params_.vehicle.threshold_s)
        v_summit=sqrt(path_length*params_.vehicle.amax);
        terminal_time=2*v_summit/params_.vehicle.amax;
        Nfe=round(terminal_time/params_.utility.traj_dt_for_resample);
        time_line=linspace(0,terminal_time,Nfe);
        time_vec1=time_line(find(time_line<=0.5*terminal_time));
        time_vec2=time_line(find(time_line>0.5*terminal_time));

        a=[ones(1,size(time_vec1,2)).*params_.vehicle.amax,ones(1,size(time_vec2,2)).*-params_.vehicle.amax];

        v_part1=time_vec1.*params_.vehicle.amax;
        v_part2=v_summit+(time_vec2-0.5*terminal_time).*-params_.vehicle.amax;
        v=[v_part1,v_part2];

        s_part1=0.5*params_.vehicle.amax*(time_vec1.^2);
        s_part2=0.5*path_length+v_summit*(time_vec2-0.5*terminal_time)+0.5*-params_.vehicle.amax*((time_vec2-0.5*terminal_time).^2);
        s=[s_part1,s_part2];
    else
        s_cruise=path_length-params_.vehicle.threshold_s;
        time_cruise=s_cruise/params_.vehicle.vmax;
        time_slope=params_.vehicle.vmax/params_.vehicle.amax;
        terminal_time=2*time_slope+time_cruise;
        Nfe=round(terminal_time/params_.utility.traj_dt_for_resample);
        time_line=linspace(0,terminal_time,Nfe);
        time_vec1=time_line(find(time_line<=time_slope));
        time_vec3=time_line(find(time_line>time_slope+time_cruise));
        time_vec1plus2=time_line(find(time_line<=time_slope+time_cruise));
        time_vec2=time_vec1plus2(find(time_vec1plus2>time_slope));

        a=[ones(1,size(time_vec1,2)).*params_.vehicle.amax,zeros(1,size(time_vec2,2)),ones(1,size(time_vec3,2)).*params_.vehicle.amax];

        v_part1=time_vec1.*params_.vehicle.amax;
        v_part2=ones(1,size(time_vec2,2)).*params_.vehicle.vmax;
        v_part3=params_.vehicle.vmax+(time_vec3-time_slope-time_cruise).*-params_.vehicle.amax;
        v=[v_part1,v_part2,v_part3];

        s_part1=0.5*params_.vehicle.amax*(time_vec1.^2);
        s_part2=0.5*params_.vehicle.amax*(time_slope^2)+...
        params_.vehicle.vmax*(time_vec2-time_slope);
        s_part3=0.5*params_.vehicle.amax*(time_slope^2)+params_.vehicle.vmax*time_cruise+...
        params_.vehicle.vmax*(time_vec3-time_slope-time_cruise)+...
        0.5*-params_.vehicle.amax*((time_vec3-time_slope-time_cruise).^2);
        s=[s_part1,s_part2,s_part3];
    end
end