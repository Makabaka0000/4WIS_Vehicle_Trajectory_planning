% Oriented Bounding box and tilt correction using PCA(Principle component analysis)
% Coded by Weichen GU 2020/4/28th
clear,clc,clf;

% load digit_100;
% idx = 25,2,64,68,69,28
% idx = 25;
% V1=reshape(train_data(idx,:),28,28)';
% [row, col] = find(V1~=0);
% data = [col 30-row];


%data = [3.7, 4.1, 4.7, 5.2, 6.0, 6.3, 9.7, 10.0, 11.0, 12.5; 1.7, 3.8, 2.9, 2.8, 4.0, 3.6, 6.3, 4.9, 3.6, 6.4]'
data = [0 0.5 1 4 4.2; 9.2 7.1 8.2 5.2 6.9]';


covariance = cov(data);                         % Obtain covariance matrix
[eigVector, eigValue] = eigs(covariance);       % Obtain eigenVector and eigenValue

mu = mean(data);                 % Obtain the mean of dataSet
%data_Mu_0 = data-mu;            % shift data to origional
coordMapped = data*eigVector;                  % mapping data to principle components(1, 2)
theta = atan2(eigVector(2,:),eigVector(1,:));       % Obtain the angle of eigen vectors(1, 2)
theta = mod(theta,pi);           % Convert theta to [0 pi)
extends = (max(coordMapped)-min(coordMapped))/2;    % Obtain the semi-length and semi-width of bounding box
%center = mean(coordMapped);
centerMapped = (max(coordMapped)+min(coordMapped))/2;
center = centerMapped/eigVector;

offSet = (extends.*[cos(theta); sin(theta)])';
weight = [-1 1; -1 -1; 1 1; 1 -1];% Left-Top, Left-Bottom, Right-Top, Right-Bottom
coord = center + weight*offSet;
coord = [coord;center];

figure(1);
plot(data(:,1),data(:,2),'b.');
num = 30;
set(gca,'XLim',[0,num]);
set(gca,'YLim',[0,num]);
axis equal;
hold on;
plot(coord(:,1),coord(:,2),'r.','Markersize',10);
line([coord(1,1) coord(2,1)],[coord(1,2) coord(2,2)]);
line([coord(1,1) coord(3,1)],[coord(1,2) coord(3,2)]);
line([coord(4,1) coord(2,1)],[coord(4,2) coord(2,2)]);
line([coord(4,1) coord(3,1)],[coord(4,2) coord(3,2)]);

draw_arrow(coord(5,:),coord(5,:)+[cos(theta(1)), sin(theta(1))],1);
a=num2str(mod(theta(1)*180/pi,180));
legend(strcat('theta = ', a, '^бу'));


% Rotation transformation
if (mod(theta(1),pi) > pi/4 && mod(theta(1),pi) < 3/4*pi )
    rotAngle = pi/2-mod(theta(1),pi);
elseif(mod(theta(1),pi) < pi/4)
    rotAngle = -mod(theta(1),pi);
else
    rotAngle = pi -mod(theta(1),pi);
end
    rotMatrix = [cos(rotAngle) sin(rotAngle); -sin(rotAngle) cos(rotAngle)];
finalAngle = theta(1)+rotAngle;

dataS =(data-center) * rotMatrix + center;
hold off;


coord = (coord-center) * rotMatrix + center;  % bbox coordinate

figure(2)
plot(dataS(:,1),dataS(:,2),'b.');
hold on;
num = 30;
set(gca,'XLim',[0,num]);
set(gca,'YLim',[0,num]);
axis equal;
plot(coord(:,1),coord(:,2),'r.','Markersize',10);
line([coord(1,1) coord(2,1)],[coord(1,2) coord(2,2)]);
line([coord(1,1) coord(3,1)],[coord(1,2) coord(3,2)]);
line([coord(4,1) coord(2,1)],[coord(4,2) coord(2,2)]);
line([coord(4,1) coord(3,1)],[coord(4,2) coord(3,2)]);
draw_arrow(center,center+[cos(finalAngle), sin(finalAngle)],1);

a=num2str(mod(finalAngle*180/pi,180));
legend(strcat('theta = ', a, '^бу'));

hold off
%http://www.mathworks.com/matlabcentral/fileexchange/27475-arrow-plotter/content/draw_arrow.m
function out = draw_arrow(startpoint,endpoint,headsize)
    %by Ryan Molecke 
    % accepts two [x y] coords and one double headsize 
    v1 = headsize*(startpoint-endpoint)/2.5; 
    theta = 22.5*pi/180; 
    theta1 = -1*22.5*pi/180; 
    rotMatrix = [cos(theta)  -sin(theta) ; sin(theta)  cos(theta)];
    rotMatrix1 = [cos(theta1)  -sin(theta1) ; sin(theta1)  cos(theta1)];  
    v2 = v1*rotMatrix; 
    v3 = v1*rotMatrix1; 
    x1 = endpoint;
    x2 = x1 + v2; 
    x3 = x1 + v3; 
    hold on; 
    fill([x1(1) x2(1) x3(1)],[x1(2) x2(2) x3(2)],[0 0 0]);% this fills the arrowhead (black) 
    plot([startpoint(1) endpoint(1)],[startpoint(2) endpoint(2)],'linewidth',1,'color',[0 0 0]);
end