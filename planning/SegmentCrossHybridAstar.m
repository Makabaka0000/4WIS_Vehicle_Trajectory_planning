function [x,y,theta,flag]=SegmentCrossHybridAstar(guided_points,IsGearShift)
    global params_
    x=[];
    y=[];
    theta=[];
    jj=1;
    forward_point=[];
    backward_point=[];
    params_.task.task_point_num=size(params_.task.task_point,1);
    flag=0;
    if IsGearShift(1,1)==1
        params_.search_index=1;
        search_point=[];
        for i=1:size(guided_points.x,2)
            forward_point=[forward_point;guided_points.x(1,i),guided_points.y(1,i),guided_points.theta(1,i)];
            search_point=[search_point;guided_points.x(1,i),guided_points.y(1,i),guided_points.theta(1,i)];
        end
        search_point=[search_point;params_.task.task_point];
        params_.search_point=search_point;
        start_config=[forward_point(jj,1),forward_point(jj,2),forward_point(jj,3)];
        while(jj<size(forward_point,1))
            end_config=[forward_point(jj+1,1),forward_point(jj+1,2),forward_point(jj+1,3)];       
            [local_x,local_y,local_theta,~]=SearchForwardHybridAstar(start_config,end_config);
            x=[x,local_x];
            y=[y,local_y];
            theta=[theta,local_theta];
            if(hypot(params_.task.xf-local_x(end),params_.task.yf-local_y(end))<0.5)
                flag=1;
                break;
            end
            start_config=[local_x(end),local_y(end),local_theta(end)];
    %         start_config=end_config;
    %         start_config=[local_x(end),local_y(end),end_config(end)];
%             jj=jj+1;  
            if jj==params_.search_index
                break;
            end
            jj=params_.search_index; 
        end       
    elseif IsGearShift(1,1)==2
        params_.search_index=1;
        search_point=[];
        for i=1:size(guided_points.x,2)
            backward_point=[backward_point;guided_points.x(1,i),guided_points.y(1,i),guided_points.theta(1,i)];
            search_point=[search_point;guided_points.x(1,i),guided_points.y(1,i),guided_points.theta(1,i)];
        end
        search_point=[search_point;params_.task.task_point];
        params_.search_point=search_point;
        start_config=[backward_point(jj,1),backward_point(jj,2),backward_point(jj,3)];
        while(jj<size(backward_point,1)) 
            end_config=[backward_point(jj+1,1),backward_point(jj+1,2),backward_point(jj+1,3)];       
            [local_x,local_y,local_theta,~]=SearchBackwardHybridAstar(start_config,end_config);
            x=[x,local_x];
            y=[y,local_y];
            theta=[theta,local_theta];
            if(hypot(params_.task.xf-local_x(end),params_.task.yf-local_y(end))<1)
                flag=1;
                break;
            end
            start_config=[local_x(end),local_y(end),local_theta(end)];
    %         start_config=end_config;
    %         start_config=[local_x(end),local_y(end),end_config(end)];
            if jj==params_.search_index
                break;
            end
            jj=params_.search_index; 
        end         
    elseif IsGearShift(1,1)==3
        end_index=IsGearShift(1,2);
        for i=1:end_index
            forward_point=[forward_point;guided_points.x(1,i),guided_points.y(1,i),guided_points.theta(1,i)];
        end
        start_config=[forward_point(jj,1),forward_point(jj,2),forward_point(jj,3)];
        while(jj<size(forward_point,1))
            end_config=[forward_point(jj+1,1),forward_point(jj+1,2),forward_point(jj+1,3)];       
            [local_x,local_y,local_theta,~]=SearchGSPForwardHybridAstar(start_config,end_config,1);
            x=[x,local_x];
            y=[y,local_y];
            theta=[theta,local_theta];
            if(hypot(params_.task.xf-local_x(end),params_.task.yf-local_y(end))<0.5)
                break;
            end
            start_config=[local_x(end),local_y(end),local_theta(end)];
    %         start_config=end_config;
    %         start_config=[local_x(end),local_y(end),end_config(end)];
            jj=jj+1;  
        end  
        for i=end_index:size(guided_points.x,2)
            backward_point=[backward_point;guided_points.x(1,i),guided_points.y(1,i),guided_points.theta(1,i)];
        end
        jj=1;
        start_config=[backward_point(jj,1),backward_point(jj,2),backward_point(jj,3)];
        while(jj<size(backward_point,1))
            end_config=[backward_point(jj+1,1),backward_point(jj+1,2),backward_point(jj+1,3)];       
            [local_x,local_y,local_theta,~]=SearchGSPBackwardHybridAstar(start_config,end_config,2);
            x=[x,local_x];
            y=[y,local_y];
            theta=[theta,local_theta];
            if(hypot(params_.task.xf-local_x(end),params_.task.yf-local_y(end))<1)
                flag=1;
                break;
            end
            start_config=[local_x(end),local_y(end),local_theta(end)];
    %         start_config=end_config;
    %         start_config=[local_x(end),local_y(end),end_config(end)];
            jj=jj+1;  
        end  
    elseif IsGearShift(1,1)==4
        end_index=IsGearShift(1,2);
        for i=1:end_index
            backward_point=[backward_point;guided_points.x(1,i),guided_points.y(1,i),guided_points.theta(1,i)];
        end  
        start_config=[backward_point(jj,1),backward_point(jj,2),backward_point(jj,3)];
        while(jj<size(backward_point,1))
            end_config=[backward_point(jj+1,1),backward_point(jj+1,2),backward_point(jj+1,3)];       
            [local_x,local_y,local_theta,~]=SearchGSPBackwardHybridAstar(start_config,end_config,1);
            x=[x,local_x];
            y=[y,local_y];
            theta=[theta,local_theta];
            if(hypot(params_.task.xf-local_x(end),params_.task.yf-local_y(end))<0.5)
                break;
            end
            start_config=[local_x(end),local_y(end),local_theta(end)];
    %         start_config=end_config;
    %         start_config=[local_x(end),local_y(end),end_config(end)];
            jj=jj+1;  
        end  
        jj=1;
        for i=end_index:size(guided_points.x,2)
            forward_point=[forward_point;guided_points.x(1,i),guided_points.y(1,i),guided_points.theta(1,i)];
        end
        start_config=[forward_point(jj,1),forward_point(jj,2),forward_point(jj,3)];
        while(jj<size(forward_point,1))
            end_config=[forward_point(jj+1,1),forward_point(jj+1,2),forward_point(jj+1,3)];       
            [local_x,local_y,local_theta,~]=SearchGSPForwardHybridAstar(start_config,end_config,2);
            x=[x,local_x];
            y=[y,local_y];
            theta=[theta,local_theta];
            if(hypot(params_.task.xf-local_x(end),params_.task.yf-local_y(end))<1.5)
                flag=1;
                break;
            end
            start_config=[local_x(end),local_y(end),local_theta(end)];
    %         start_config=end_config;
    %         start_config=[local_x(end),local_y(end),end_config(end)];
            jj=jj+1;  
        end  
    end
end