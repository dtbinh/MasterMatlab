clear;
close all;

X = [100 0 20]';

X_hat0 = [0 0 0 ];

p1 = [0 2.5 -1]';
p2 = [0 -2.5 -1]';
p3 = [-4 -1 2]';
p4 = [-5 1 3]';
% p5 = [0 0 1]';
% p6 = [1 0 0]';
P = [p1 p2 p3 p4];
[~,K]=size(P);
W = eye(K);
W(1,1) = 0.02;
W(2,2) = 0.002;
W(3,3) = 0.01;
W(4,4) = 0.01;
% W(5,5) = 10;
N = 10;
x_bar = zeros(3,N);
x_bar(:,1) = X_hat0;
H = zeros(K,3);
HH = zeros(K,1);
h = zeros(K,1);
Y = zeros(K,1);
I = zeros(3,3);
for i=1:N
    for k=1:K
        Y(k) = norm(X-P(:,k)); %Create measurement
        Y(k) = Y(k) + 0.1*wgn(1,1,0);
    end
    for u=1:20
    for k=1:K
        
        r = norm(x_bar(1:3,i)-P(:,k));
        H(k,:) = [(x_bar(1,i)-P(1,k))/r (x_bar(2,i)-P(2,k))/r (x_bar(3,i)-P(3,k))/r ];
%         hh(:,:,k) = 1/r^(3/2)*[r+2*(x_bar(1,i)-P(1,k))^2 2*(x_bar(1,i)-P(1,k))*(x_bar(2,i)-P(2,k)) 2*(x_bar(1,i)-P(1,k))*(x_bar(3,i)-P(3,k)); 
%             2*(x_bar(1,i)-P(1,k))*(x_bar(2,i)-P(2,k)) r+2*(x_bar(2,i)-P(2,k))^2 2*(x_bar(2,i)-P(2,k))*(x_bar(3,i)-P(3,k))
%             2*(x_bar(1,i)-P(1,k))*(x_bar(3,i)-P(3,k)) 2*(x_bar(2,i)-P(2,k))*(x_bar(3,i)-P(3,k)) r+2*(x_bar(3,i)-P(3,k))^2];
%         HH(k,1) = trace(I*hh(:,:,k));
        h(k) = r ;
    end
    I = inv(H'/W*H);
    x_bar(:,i)=(H'/W*H)\H'/W*(Y-h+H*x_bar(:,i));
    end
    x_bar(:,i+1) = x_bar(:,i);
end
I
figure(1);
subplot(3,1,1);
plot(x_bar(1,:));
grid on;
subplot(3,1,2);
plot(x_bar(2,:));
grid on;
subplot(3,1,3);
plot(x_bar(3,:));
grid on;
% figure(2)
% plot(x_bar(4,:));
figure(3);
plot3(X(2),X(1),X(3),'x');
hold on;
plot3(p1(2),p1(1),p1(3),'rx');
plot3(p2(2),p2(1),p2(3),'rx');
plot3(p3(2),p3(1),p3(3),'rx');
plot3(p4(2),p4(1),p4(3),'rx');