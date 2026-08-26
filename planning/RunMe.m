    close all; clc; clear global params_; clear all;
global params_ 
for ii = 1:1
%     ii=1;
    params_.user.case_id = ii;
    InitializeParams();
    DrawParkingScenario();
    SearchTrajectoryViaFTHA();
    OptimizeTrajectoryViaLIOM();
end 
DrawParkingScenario();
plot(params_.limo.x,params_.limo.y,'LineWidth',2);

distribution=params_.distribution;
for j=1:size(distribution,1)
    for i=1:size(distribution(j).x,2)
        plot(distribution(j,1).x(:,i).mu,distribution(j,1).y(:,i).mu,'r*');
        hold on;
    end
end

