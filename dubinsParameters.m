function [Pchi,PN] = dubinsParameters(Ocs,Ocf,Rs,Rf,TurnS,TurnF)

Xsc = Ocs(1);
Ysc = Ocs(2);

Xfc = Ocf(1);
Yfc = Ocf(2);

cbx = Xsc;
cax = Xfc-cbx;
cby = Ysc;
cay = Yfc-cby;

dc = sqrt(cax^2+cay^2);

if abs(Rf-Rs)>abs(dc)
    disp('Advarsel: Umulig Dubin');
end
alpha = asin((Rf-Rs)/abs(dc));

beta = atan2(Yfc-Ysc,Xfc-Xsc);

thetas = turn(TurnS,alpha,beta);

thetaf = turn(TurnF,alpha,beta);

Pchi = [Xsc+Rs*cos(thetas);Ysc + Rs*sin(thetas)];
PN = [Xfc+Rf*cos(thetaf);Yfc+Rf*sin(thetaf)];

end