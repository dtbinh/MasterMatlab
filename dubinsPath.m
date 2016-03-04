function [Path] = dubinsPath(Xs,Xf,Rs,Rf,N)
% Create dubins path between two positions.
% Start pose
% Xs(1) = north (m)
% Xs(2) = east (m)
% Xs(3) = down (m)
% Xs(4) = heading (m)
% Rs turning circle start
% End pose
% Xf(1) = north (m)
% Xf(2) = east (m)
% Xf(3) = down (m)
% Xf(4) = heading (m)
% Rf turning circle end
% N Number of turing points

% Define start turning circle
if atan2(Xs(2)-Xf(2),Xs(1)-Xf(2))<0
    RightS = false;
    Xsc = Xs(1)-Rs*cos(Xs(4)-pi/2);
    Ysc = Xs(2)-Rs*sin(Xs(4)-pi/2);
else
    RightS = true;
    Xsc = Xs(1)-Rs*cos(Xs(4)+pi/2);
    Ysc = Xs(2)-Rs*sin(Xs(4)+pi/2);
end
RightS
%Define end turning circle. First find whitch side
if atan2(Xs(2)-Xf(2),Xs(1)-Xf(2))<0
    RightF = false;
    Xfc = Xf(1)-Rf*cos(Xf(4)-pi/2);
    Yfc = Xf(2)-Rf*sin(Xf(4)-pi/2);
     
else
    RightF = true;
    Xfc = Xf(1)-Rf*cos(Xf(4)+pi/2);
    Yfc = Xf(2)-Rf*sin(Xf(4)+pi/2);
end
% Radius of secound end circle
Rsec = abs(Rs-Rf);

% Define c
cbx = Xsc;
cax = Xfc-cbx;
cby = Ysc;
cay = Yfc-cby;

dc = sqrt(cax^2+cay^2);

% Tmerk is perpendicular to c form Of, and connect in Csec;
ttmerk = sqrt(Rsec^2/(cax^2+cay^2));
Tmerk = [cay*ttmerk+Xfc;-cax*ttmerk+Yfc];
ttPn = sqrt(Rf^2/(cax^2+cay^2));
PN = [cay*ttPn+cbx;-cax*ttPn+cby];
OSTmerkbx = Xsc;
OSTmerkax = Tmerk(1)-OSTmerkbx;
OSTmerkby = Ysc;
OSTmerkay = Tmerk(2)-OSTmerkby;

% Find the exit tangent point
ttPchi = sqrt(Rs^2/(cax^2+cay^2));
Pchi = [cay*ttPchi+cbx;-cax*ttPchi+cby];

alpha = asin((Rf-Rs)/abs(dc));

beta = atan2(Yfc-Ysc,Xfc-Xsc);

thetas = turn(RightS,alpha,beta);

thetaf = turn(RightF,alpha,beta);

Pchi = [Xsc+Rs*cos(thetas);Ysc + Rs*sin(thetas)];
PN = [Xfc+Rf*cos(thetaf);Yfc+Rf*sin(thetaf)];

% thetaf1 = turn(true,alpha,beta);
% thetaf2 = turn(false,alpha,beta);
% PN1 = [Xfc+Rf*cos(thetaf1);Yfc+Rf*sin(thetaf1)];
% PN2 = [Xfc+Rf*cos(thetaf2);Yfc+Rf*sin(thetaf2)];
% 
% theta011 = atan2(PN1(2)-Yfc,PN1(1)-Xfc);
% theta012 = atan2(PN2(2)-Yfc,PN2(1)-Xfc);
% theta11 = atan2(Xf(2)-Yfc,Xf(1)-Xfc);
% difftheta1 = abs(pipi(theta011-theta11));
% difftheta2 = abs(pipi(theta012-theta11));
% diff = [difftheta1 difftheta2];
% PNx = [PN1 PN2];
% [~,it]=min(diff);
% PN = PNx(:,it);


% atan2(Xs(2)-Yfc,Xs(1)-Xfc)*180/pi
% atan2(PN(2)-Yfc,PN(1)-Xfc)*180/pi
% atan2(Xf(2)-Yfc,Xf(1)-Xfc)*180/pi
% atan2(Pchi(2)-Xf(2),Pchi(1)-Xf(1))*180/pi
% atan2(-Pchi(2)+PN(2),-Pchi(1)+PN(1))*180/pi
% atan2(Xf(2)-Yfc,Xf(1)-Xfc)*180/pi-atan2(PN(2)-Yfc,PN(1)-Xfc)*180/pi
% if abs(atan2(Xs(2)-Yfc,Xs(1)-Xfc))<pi && abs(atan2(Xf(2)-Yfc,Xf(1)-Xfc)-atan2(PN(2)-Yfc,PN(1)-Xfc))>pi
%     changePN = true;
% else
%     changePN = false;
% end
% changePN
% if changePN
%     thetaf = turn(~RightF,alpha,beta);
%     PN = [Xfc+Rf*cos(thetaf);Yfc+Rf*sin(thetaf)];
% end
% atan2(PN(2)-Yfc,PN(1)-Xfc)*180/pi
% atan2(Xf(2)-Yfc,Xf(1)-Yfc)*180/pi
% atan2(Xf(2)-Yfc,Xf(1)-Yfc)*180/pi-atan2(PN(2)-Yfc,PN(1)-Xfc)*180/pi

% 


theta0 = atan2(Xs(2)-Ysc,Xs(1)-Xsc)
theta1 = atan2(Pchi(2)-Ysc,Pchi(1)-Xsc)
sign(theta1)
sign(theta0)
pipi(theta1-pi)*180/pi
if RightS
    if ((pipi(theta1-pi)>=theta0 ) || (theta0>theta1 &&sign(theta1)==sign(theta0)))
        theta = 0:-(abs(pipi(theta1-theta0))/(N-1)):-abs(pipi(theta1-theta0));
    else
        theta = 0:-((2*pi-abs(pipi(theta1-theta0)))/(N-1)):-(2*pi-abs(pipi(theta1-theta0)));
    end
else
    if ((pipi(theta1-pi)<=theta0 && sign(pipi(theta1-pi))==sign(theta0)) || (theta0<theta1 && (sign(theta1)==sign(theta0) || theta0==0)))
        theta = 0:abs((pipi(theta1-theta0))/(N-1)):abs(pipi(theta1-theta0));
    else
        theta = 0:(2*pi-abs((pipi(theta1-theta0))))/(N-1):2*pi-abs(pipi(theta1-theta0));
    end
    
end
% theta = 0:((theta1-theta0))/(N-1):theta1-theta0;
arc1 = [Xsc+Rs*cos(theta0+theta);Ysc+Rs*sin(theta0+theta)];
Path = arc1;
theta01 = atan2(PN(2)-Yfc,PN(1)-Xfc);
theta11 = atan2(Xf(2)-Yfc,Xf(1)-Xfc);
% && sign(pipi(theta11-pi))==sign(theta01)

if RightF
    %counter clock wise rotation
    if ((pipi(theta11-pi)>=theta01 ) || (theta01>theta11 &&sign(theta11)==sign(theta01)))
        theta1 = 0:-(abs(pipi(theta11-theta01))/(N-1)):-abs(pipi(theta11-theta01));
    else
        theta1 = 0:-((2*pi-abs(pipi(theta11-theta01)))/(N-1)):-(2*pi-abs(pipi(theta11-theta01)));
    end
else
    if ((pipi(theta11-pi)<=theta01 && sign(pipi(theta11-pi))==sign(theta01)) || (theta01<theta11 && sign(theta11)==sign(theta01)))
        theta1 = 0:abs((pipi(theta11-theta01))/(N-1)):abs(pipi(theta11-theta01));
    else
        theta1 = 0:(2*pi-abs((pipi(theta11-theta01))))/(N-1):2*pi-abs(pipi(theta11-theta01));
    end
end
% theta1 = 0:(theta11-theta01)/(N-1):theta11-theta01;
arc2 = [Xfc+Rf*cos(theta01+theta1);Yfc+Rf*sin(theta01+theta1)];
Path = [Path arc2];

% Debuging 
s = 0:0.01:1;
OST = [OSTmerkax*s+OSTmerkbx;OSTmerkay*s+OSTmerkby];
plot(Path(2,:),Path(1,:));
hold on;
plot(Xs(2),Xs(1),'yo');
plot(Xf(2),Xf(1),'yx');
% plot(OST(2,:),OST(1,:),'b');
plot(Tmerk(2),Tmerk(1),'bx');
plot(Ysc,Xsc,'cx');
plot(PN(2),PN(1),'ro')
plot(Pchi(2),Pchi(1),'rx');
axis equal
end