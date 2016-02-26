%% Plots the Dubins path to be used with a climb rate guidance law
close all;
clear
%clear all; clc; close all
warning('off','all')

xx = 1; % #Define
yy = 2; % #Define

ground_level = 0;
attack_angle = 4*(pi/180);
descend_angle = 4*(pi/180);
dist_behind = 100;
dist_infront = -100;
net_height = 3;
height = 30;
path_z = zeros(1,5);
hor_dist = zeros(1,5);
min_turn_radius = 50;

path_z(1,5) = net_height-dist_behind*tan(attack_angle);
path_z(1,4) = net_height;
path_z(1,3) = path_z(1,4)-dist_infront*tan(attack_angle);
path_z(1,2) = height;
path_z(1,1) = path_z(1,2);
hor_dist(1,5) = dist_behind;
hor_dist(1,4) = 0;
hor_dist(1,3) = dist_infront;
hor_dist(1,2) = hor_dist(1,3)-(height-path_z(1,3))/tan(descend_angle)+ground_level/tan(descend_angle);
hor_dist(1,1) = hor_dist(1,2)-2*min_turn_radius;
% path_z = [30 30 2 0 -1];
% hor_dist = [0 75 75+150 75+150+50 75+150+50+25];

net_hor = hor_dist(4);
WP = [hor_dist; path_z];

figure(1); hold on
% plot(hor_dist-net_hor,path_z,'r*-')
plot(hor_dist,path_z,'r*-')
grid on

% x_all  = [hor_dist(1)-net_hor];
% y_all  = [path_z(1)];
% dy_all = [0];
% 
% %% Circle arcs
% %max_pitch = 5.5; [deg] (ved 2/20 m/s)
% %max_pitch = 9.5; [deg] (ved 2/12 m/s)
% R = 500;
% 
% num_WP = length(WP(xx,:));
% 
% x_prev = WP(xx,1)-net_hor;
% 
% for i=1:(num_WP-3)
%     a = norm(WP(:,i)-WP(:,i+2));
%     b = norm(WP(:,i)-WP(:,i+1));
%     c = norm(WP(:,i+1)-WP(:,i+2));
%     cos_2_alpha = (-a*a+b*b+c*c)/(2*b*c);
%     
%     alpha = acos(cos_2_alpha)/2;
%     
%     R_tangent = R/tan(alpha);
%     
%     theta1 = atan2(WP(yy,i+1)-WP(yy,i),WP(xx,i+1)-WP(xx,i));
%     theta2 = atan2(WP(yy,i+2)-WP(yy,i+1),WP(xx,i+2)-WP(xx,i+1));
%     
%     x1 = WP(xx,i+1) - R_tangent*cos(theta1);
%     y1 = WP(yy,i+1) - R_tangent*sin(theta1);
%     
%     x2 = WP(xx,i+1) + R_tangent*cos(theta2);
%     y2 = WP(yy,i+1) + R_tangent*sin(theta2);
%     
%     c = sqrt((x1-x2)^2 + (y1-y2)^2);
%     cosAlpha = (c^2)/(2*R*c);
%     
%     A = [x1 y1];
%     B = [x2 y2];
%     u_AB = (B - A)/c; % Unit vector from first to second center
%     pu_AB = [u_AB(2), -u_AB(1)]; % Perpendicular vector to unit vector
% 
%     intersect_1 = A + u_AB * (R*cosAlpha) + pu_AB * (R*sqrt(1-cosAlpha^2));
%     intersect_2 = A + u_AB * (R*cosAlpha) - pu_AB * (R*sqrt(1-cosAlpha^2));
%     
%     if ((WP(xx,i+1)-intersect_1(1))^2 + (WP(yy,i+1)-intersect_1(2))^2 > R^2)
%         xc = intersect_1(1);
%         yc = intersect_1(2);
%     else
%         xc = intersect_2(1);
%         yc = intersect_2(2);
%     end
%     
% %     syms xc yc
% %     [xc,yc] = solve((x1-xc)^2 + (y1-yc)^2 == R^2, (x2-xc)^2 + (y2-yc)^2 == R^2, (WP(xx,i+1)-xc)^2 + (WP(yy,i+1)-yc)^2 > R^2, xc, yc);
% %     xc = double(xc);
% %     yc = double(yc);
% %     
% %     xc = xc(1)
% %     yc = yc(1)
%     
%     x1 = x1-net_hor;
%     x2 = x2-net_hor;
%     xc = xc-net_hor;
%     
%     numPoints = 1000;
%     x  = linspace(x1,x2,numPoints);
%     if y1-yc > 0
%         sgn = 1;
%     else
%         sgn = -1;
%     end
%     y  = sgn*sqrt(R^2-(x-xc).^2);  
%     %y  = sgn*sqrt(R^2-(-R:R/500:(R-R/500)).^2); 
%     plot(x,y+yc,'black-','linewidth',1);
%     %plot((-R:R/500:(R-R/500))+xc,y+yc,'black-','linewidth',1);
%     
%     
%     x_all = [x_all x1 x x2];
%     y_all = [y_all y1 (y+yc) y2];
%     
%     dy2 = zeros(1,numPoints);
%     dy2(1) = theta1;
%     for j=1:(length(y)-1)
%         dy2(j+1) = atan2(y(j+1)-y(j),x(j+1)-x(j));
%     end
%     dy2(end) = theta2;
%     
%     %dy = dy2;
%     dy = -sgn*(x-xc)./sqrt(R^2-(x-xc).^2);
%     dy_all = [dy_all theta1 dy2 theta2];
%     
%     figure(2); hold on;
%     plot([x_prev x1],rad2deg([theta1 theta1]),'black-','linewidth',1); %Straight lines
%     plot(x,rad2deg(dy),'black-','linewidth',1);
%     figure(1); hold on;
%     x_prev = x2;
%     
%     plot(x1,y1,'b*')
%     plot(x2,y2,'g*')
%     %plot(xc,yc,'black*')
% end
% legend('Path lines','Path circle arcs')
% xlabel('Longitudinal distance [m]'); ylabel('Height [m]');
% 
% figure(2); hold on;
% plot([x_prev WP(xx,end)-net_hor],rad2deg([theta2 theta2]),'black-','linewidth',1);
% grid on; legend('Desired climb rate (picewise)')
% xlabel('Longitudinal distance [m]'); ylabel('Climb rate [m/s]');
% 
% x_all = [x_all hor_dist(end)-net_hor];
% y_all = [y_all path_z(end)];
% dy_all = [dy_all theta2];
% 
% figure(3); hold on
% plot(hor_dist-net_hor,path_z,'b--*','linewidth',2)
% plot(x_all,y_all,'r-','linewidth',2);
% grid on; legend('Original path','Dubins path')
% xlabel('Longitudinal distance [m]'); ylabel('Height [m]');
% title('Dubins landing path')
% 
% %%
% figure(4);
% % plot(x_all,rad2deg(dy_all),'b-','linewidth',1);
% plot(x_all,dy_all,'b-','linewidth',2);
% grid on; legend('Desired climb ratio')
% xlabel('Longitudinal distance [m]'); ylabel('Climb ratio [m/m]');
% title('Derivative of Dubins landing path')
% 
% %%
% figure(5); hold on
% [hAx,l1,l2] = plotyy(x_all,y_all,x_all,dy_all)
% legend('Dubins Path','Dubins Path Derivative')
% xlabel('Longitudinal distance [m]');
% ylabel(hAx(1),'Desired Height [m]') % left y-axis
% ylabel(hAx(2),'Desired Climb Rate Ratio [m/m]') % right y-axis
% xlim(hAx(1),[-275 25])
% xlim(hAx(2),[-275 25])
% ylim(hAx(1),[-5 35])
% ylim(hAx(2),[-0.7/3 0.1/3])
% grid(hAx(1),'on')
% grid(hAx(2),'on')
% title('Dubins landing path')
% 
% %%
% dy2_all = zeros(1,length(x_all));
% for i=1:length(dy2_all)-1
%     dy2_all(i) = (y_all(i+1)-y_all(i))/(x_all(i+1)-x_all(i));
% end
% dy2_all(end) = dy2_all(end-1);
% figure(6); hold on; grid on
% plot(x_all,dy_all,'b')
% plot(x_all,dy2_all,'r')
% legend('Old (angle)','New (dy/dx)')