close all;
clear all;
%% Day 1
meanHeightError = [1.5 2.6 0.9 0.1 1.7 1.3 1.8 1.2 1.9 1.5 1.5]';
meanCrossTrack = [6.1 6.7 5.5 2.8 2.0 6.8 9.1 8.2 5.9 4.4 1.3]';
disp('Mean height error')
mean(meanHeightError)
disp('Mean cross track error')
mean(meanCrossTrack)
%% Day 1 net passing
% heightError = [2.8 2.7 0.9 0 0.8 2.1 0.7 -1.5 1.9 0.3 -1.3]';
% crossTrack  = [2.1 -4.5 -1.6 5.4 5.3 -1.6 2.3 -5.4 0.8 1.1 0.2]';

%% Day 2 net passing
heightError = [14.4 1.3 1.1 1.4 1.1 2.0 2.32 7]';
crossTrack  = [0.1 0.6 -0.2 0.1 0.1 -0.2 0.2 0.3]';

Net = [1.5 -2.5;
        -1.5 -2.5;
        -1.5 2.5;
        1.5 2.5;
        1.5 -2.5];
    
figure(1)
plot(Net(:,2),Net(:,1))
grid on;
hold on;
plot(crossTrack,heightError,'rx')
axis('equal')
legend('Net landing box','UAV position when passing the net');
xlabel('Cross track error [m]');
ylabel('Height error with respect to net center [m]')