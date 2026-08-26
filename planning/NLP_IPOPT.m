function[x_opt,y_opt,theta_opt,v_opt,delta_opt,a_opt,w_opt] =  NLP_IPOPT(x,y,theta,pattern)
global NLP_
x_rest=[];y_rest=[];theta_rest=[];
v_rest=[];delta_rest=[];a_rest=[];w_rest=[];terminal_time=0;
x_opt=[];y_opt=[];theta_opt=[];v_opt=[];delta_opt=[];a_opt=[];w_opt=[];

pattern_change=getpatternindex(pattern);
index=1;
while(index<=size(pattern_change))
    index_1=pattern_change(index);
    index_2=pattern_change(index+1);
    x1 = x(index_1:index_2);
    y1 = y(index_1:index_2);
    theta1 = theta(index_1:index_2);
    [x_res,y_res,theta_res,v_res,delta_res,a_res,w_res,terminal_time1] = resample(x1,y1,theta1);
    [x1_opt,y1_opt,theta1_opt,v1_opt,delta1_opt,a1_opt,w1_opt] = optimize_ipopt(x_res,y_res,theta_res,v_res,delta_res,a_res,w_res,terminal_time1);
    if (NLP_.resample.display_xy)
        x_rest=[x_rest,x_res];y_rest=[y_rest,y_res];theta_rest=[theta_rest,theta_res];
        v_rest=[v_rest,v_res];delta_rest=[delta_rest,delta_res];a_rest=[a_rest,a_res];
        w_rest=[w_rest,w_res];terminal_time=terminal_time+terminal_time1;
    end
    x_opt=[x_opt,x1_opt];y_opt=[y_opt,y1_opt];theta_opt = [theta_opt,theta1_opt];
    v_opt=[v_opt,v1_opt];delta_opt=[delta_opt,delta1_opt];a_opt=[a_opt,a1_opt];w_opt=[w_opt,w1_opt];
    index=index+2;
end
% 可视化重采样点
if (NLP_.resample.display_xy)
    plot(x_res,y_res,'b','LineWidth',2);
end
end

%% 获取模式转换的索引
function pattern_change = getpatternindex(pattern)
pattern_length = length(pattern);
pattern_change = 1;
index = 2;
% 要剔除原地转向的点
while(index <= pattern_length)
    if (pattern(index) ~= pattern(index-1))
        if (pattern(index)==1)
            pattern_change=[pattern_change,index-1,index+1];
        elseif((pattern(index)~=1)&&(pattern(index-1)~=1))
            pattern_change=[pattern_change,index-1,index];
        end
    end
    index = index + 1;
end
ii = size(pattern_change,2);
if pattern(end)==1
    pattern_change=pattern_change(1:ii-2);
end
pattern_change(end + 1) = pattern_length;
end

