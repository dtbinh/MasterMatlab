clear;
close all;
position = 1;
X0 = [100 0 20]';
N = 10000;
X = zeros(3,N);
X(:,1) = X0;
V = zeros(3,N);
X_hat0 = [0 0 0 ];
Kp = 100*eye(3);
Kd = 200*eye(3);
WP = [90 0 20;80 0 20;70 0 20;60 0 18;50 0 15;40 0 12;30 0 12;30 0 10;20 0 6;10 0 4;0 0 0];
A = [zeros(3,3) eye(3);-Kp -Kd];
f = 0.01;
for n=1:4
    switch position
        case 1
            p1 = [0 2.5 -1]';
            p2 = [0 -2.5 -1]';
            p3 = [-8 -0.5 10]';
            p4 = [-8 0.5 10]';
            color = 'b';
        case 2
            p1 = [0 -3 0]';
            p2 = [0 -3 0]';
            p3 = [1 -4 -2]';
            p4 = [1 4 2]';
            color = 'r';
        case 3
            p1 = [1 0 -2]';
            p2 = [-7 -4 0]';
            p3 = [-8 0 10]';
            p4 = [-7 4 0]';
            color = 'g';
        case 4
            p1 = [1 -4 -2]';
            p2 = [-8 3 10]';
            p3 = [0 3 1.5]';
            p4 = [-2 -2 5]';
            color = 'c';
    end
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
    
    x_bar = zeros(3,N);
    es = zeros(3,N);
    e = zeros(3,N);
    x_bar(:,1) = X_hat0;
    H = zeros(K,3);
    HH = zeros(K,1);
    h = zeros(K,1);
    Y = zeros(K,1);
    I = zeros(3,3);
    tt = 1;
    for i=1:N
        for k=1:K
            Y(k) = norm(X(:,i)-P(:,k)); %Create measurement
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
        e(:,i) = X(:,i)-WP(tt,:)';
        X(:,i+1) = X(:,i) + f*V(:,i);
        V(:,i+1) = V(:,i) + f*(-Kp*e(:,i) - Kd*V(:,i));
        x_bar(:,i+1) = x_bar(:,i);
        es(:,i) = X(:,i)-x_bar(:,i);
        if mod(i,1000)==0
            tt = tt + 1;
        end
    end
    I
    figure(1);
    subplot(3,1,1);
    plot(es(1,:),color);
    hold on;
    grid on;
    subplot(3,1,2);
    plot(es(2,:),color);
    hold on;
    grid on;
    subplot(3,1,3);
    plot(es(3,:),color);
    hold on;
    grid on;
    % figure(2)
    % plot(x_bar(4,:));
    figure(2);
    subplot(3,1,1);
    plot(e(1,:),color);
    grid on;
    subplot(3,1,2);
    plot(e(2,:),color);
    grid on;
    subplot(3,1,3);
    plot(e(3,:),color);
    grid on;
    position = position + 1;
end
figure(3);
plot3(X(2),X(1),X(3),'x');
hold on;
plot3(p1(2),p1(1),p1(3),'rx');
plot3(p2(2),p2(1),p2(3),'rx');
plot3(p3(2),p3(1),p3(3),'rx');
plot3(p4(2),p4(1),p4(3),'rx');