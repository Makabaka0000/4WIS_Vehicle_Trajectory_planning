    close all; clc; clear global params_; clear all;
global params_ 
for ii = 3:3
%     ii=1;
    params_.user.case_id = ii;
%     InitializeParams();
    MyInitPar();
%     [x_test,y_test,theta_test]=SearchAStarPath([params_.task.x0,params_.task.y0,params_.task.theta0],[params_.task.xf,params_.task.yf,params_.task.thetaf]);
%     DrawParkingScenario();
%     DrawTrajFootprints(params_.task.xf,params_.task.yf,params_.task.thetaf);
%     plot(x_test,y_test,'LineWidth',2);
    CrossHybridstar();


%     [IsGearShift,gsp_point]=GetMapInformation();
%     SearchTrajectory4WIS(IsGearShift,gsp_point);


%     load('Params.mat')
%     OptimizeTrajectory4WIS();
%     OptimizeTrajectoryViaLIOM();
%     if(params_.limo.is_successful)
%         x=params_.limo.x;
%         y=params_.limo.y;
%         theta=params_.limo.theta;
%         v=params_.limo.v;
%         a=params_.limo.a;
%         phy=params_.limo.phy;
%         w=params_.limo.w;
%         terminal_time=params_.limo.terminal_time;
%         save(strcat('E:\MATLAB 2019a\bin\mycor\dilemma\hybrid\res',num2str(ii)),'x','y','theta','v','a','phy','w','terminal_time','computation_time');
%     end
end 
% DrawParkingScenario();
% plot(params_.ha_result.x,params_.ha_result.y,'LineWidth',2);
% DrawTrajFootprints(params_.ha_result.x,params_.ha_result.y,params_.ha_result.theta);
% hold on
% spin=params_.ha_result.spin;
% for i=1:size(spin,1)
%     t=linspace(spin(i,3),spin(i,4),50);
%     r=params_.vehicle.length/2;
%     center=[spin(i,1),spin(i,2)];
%     x=cos(t).*r+center(1,1);
%     y=sin(t).*r+center(1,2);
%     plot(x,y,'LineWidth',2);
%     Arrow([spin(i,1),spin(i,2)],[spin(i,1)+cos(spin(i,3)),spin(i,2)+sin(spin(i,3))],'Length',1.6,'BaseAngle',9,'TipAngle',1.6,'Width',0.2);
%     Arrow([spin(i,1),spin(i,2)],[spin(i,1)+cos(spin(i,4)),spin(i,2)+sin(spin(i,4))],'Length',1.6,'BaseAngle',9,'TipAngle',1.6,'Width',0.2);
%     hold on
% end



