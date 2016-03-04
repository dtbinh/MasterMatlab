function [Path] = glideSpiral(Path,OF,Rf,RightF,h1,correctHeight,N,glideangle)
theta0 = atan2(Path(2,end)-OF(2),Path(1,end)-OF(1));
WP4 = Path(:,end);
if RightF
    theta = 0:-(2*pi)/(N-1):-2*pi;
else
    theta = 0:(2*pi)/(N-1):2*pi;
end
if (~correctHeight)
    WPS0 = Path(:,end);
    xnn = OF(1)+Rf*cos(theta0+theta(2));
    ynn = OF(2)+Rf*sin(theta0+theta(2));
    D = sqrt((xnn-WPS0(1))^2+(ynn-WPS0(2))^2);
    znn = WPS0(3)+D*tan(glideangle);
    WPS1 = [xnn;ynn;znn];
    n = 3;
    tt = length(Path) +1;
    Path(:,tt) = WPS0;
    tt = tt + 1;
    Path(:,tt) = WPS1;
    while(~correctHeight)
        if abs(atan2(h1-WPS1(3),D))<abs(glideangle)
            glideangle = atan2(h1-WPS1(3),D);
            correctHeight = true;
        end
        WPS0 = WPS1;
        xnn = OF(1)+Rf*cos(theta0+theta(n));
        ynn = OF(2)+Rf*sin(theta0+theta(n));
        D = sqrt((xnn-WPS0(1))^2+(ynn-WPS0(2))^2);
        znn = WPS0(3)+D*tan(glideangle);
        WPS1 = [xnn;ynn;znn];
        tt = tt+1;
        Path(:,tt) = WPS1;
        n = n+1;
        if n>N
           n = 2;
       end
    end
    thetaH0 = atan2(WPS1(2)-OF(2),WPS1(1)-OF(1));
    theta11 = atan2(WP4(2,end)-OF(2),WP4(1,end)-OF(1));
    if RightF
        %counter clock wise rotation
        if ((pipi(theta11-pi)>=thetaH0 && sign(pipi(theta11-pi))==sign(thetaH0)) || (thetaH0>theta11 &&sign(theta11)==sign(thetaH0)))
            theta1 = 0:-(abs(pipi(theta11-thetaH0))/(N-1)):-abs(pipi(theta11-thetaH0));
        else
            theta1 = 0:-((2*pi-abs(pipi(theta11-thetaH0)))/(N-1)):-(2*pi-abs(pipi(theta11-thetaH0)));
        end
    else
        if ((pipi(theta11-pi)<=thetaH0 && sign(pipi(theta11-pi))==sign(thetaH0)) || (thetaH0<theta11 && sign(theta11)==sign(thetaH0)))
            theta1 = 0:abs((pipi(theta11-thetaH0))/(N-1)):abs(pipi(theta11-thetaH0));
        else
            theta1 = 0:(2*pi-abs((pipi(theta11-thetaH0))))/(N-1):2*pi-abs(pipi(theta11-thetaH0));
        end
    end
    if pipi(theta11-thetaH0)==0
        theta1 = 0;
    end
    arc = [OF(1)+Rf*cos(thetaH0+theta1);OF(2)+Rf*sin(thetaH0+theta1);ones(1,length(theta1))*Path(3,end)];
    Path = [Path arc];
end
end