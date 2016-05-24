figure(1)
plot(yDubin,xDubin);
hold on;
grid on;
figure(2);
plot3(yDubin,xDubin,-zDubin);
grid on;
hold on;
figure(5)
plot3(PathY,PathX,-PathZ);
hold on;
plot3(Estimated.PathE,Estimated.PathN,-Estimated.PathD);
grid on;
figure(6)
plot(PathY,PathX);
hold on;
plot(Estimated.PathE,Estimated.PathN,'r')
grid on;
figure(7)
plot(alongTrack(1:j))
hold on;
grid on;
figure(8)
plot(crossTrack(1:j))
grid on;
hold on;
figure(9)
plot(-PathZ)
figure(10)
plot(PathState.timestamp-PathState.timestamp(1),PathState.crossTrack)
hold on;
figure(11)
plot(Estimated.timestamp-Estimated.timestamp(1),Estimated.base_height-Estimated.z);
hold on;
plot(DesiredZ.timestamp-DesiredZ.timestamp(1),DesiredZ.value,'r');
grid on;