clear;
close all;
Hardware = true;
filename = 'Agdenes_1juni/114124_land_fixedwing_4_run2/mra/data';

state = extractNavState(filename,Hardware);

figure(1)
plot(state.Estimated.DisE,state.Estimated.DisN);
grid on;
figure(2)
plot(state.Rtk.timestamp-state.Rtk.timestamp(1),state.Rtk.type)
grid on;
hold on;
plot(state.Navsources.timestamp-state.Navsources.timestamp(1) ,state.Navsources.maskValue,'--r')
legend('RTK-GNSS','UAV navigation source')
ylabel(gca,'RTKGNSS solution type');
set(gca,'Box','off','YTick',[0 2 3],'YTickLabel',{'NONE', 'FLOAT', 'FIX'});
axesPosition = get(gca,'Position');
HNewAxes = axes('Position',axesPosition,...
            'Color','none',...
            'YAxisLocation','right',...
            'YLim',[0 3],...
            'YTick',[0 1],...
            'YTickLabel',{'EXT', 'RTK'},...
            'XTick',[],...
            'Box','off');
ylabel(HNewAxes,'Navigation source');
figure(3)
plot(state.Estimated.timestamp-state.Estimated.timestamp(1),state.Estimated.base_height-state.Estimated.z)
hold on;
grid on;
plot(state.External.timestamp-state.External.timestamp(1),state.External.base_height-state.External.z,'--r');
plot(state.Rtk.timestamp-state.Rtk.timestamp(1),state.Rtk.base_height-state.Rtk.d,'--g');
ylabel('Height (WGS84) [m]');
xlabel('Time [s]');
legend('UAV height','External navigation data','RTK-GNSS');
figure(4)
plot(state.Rtk.timestamp-state.Rtk.timestamp(1),state.Rtk.satellite);
grid on;
ylabel('Number of valid satellites');
xlabel('Time [s]');
legend('Valid satellites in the RTK-GNSS system');
ylim([0 15]);
% ax1 = axes;
% ax2 = axes;
% axes(ax1)
% set(gca,'YTick',[0 2 3],'YTickLabel',{'NONE', 'FLOAT', 'FIX'},);
% axes(ax2)
% set(ax2,'color','none','YAxisLocation','right');
% set(gca,'YTick',[0 1],'YTickLabel',{'EXTERNAL', 'RTK-GNSS'},'XTick',[],'YLim',[0 1]);
% ax1 = gca;
% ax1.YTick = [0 2 3];
% ax1.YTickLabel = {'NONE', 'FLOAT', 'FIX'};
% ax1_pos = ax1.Position;
% ax2 = axes('Position',ax1_pos,...
%         'XAxisLocation','bottom',...
%         'YAxisLocation','right',...
%         'Color','none');
% ax2.YTick = [0 1];
% ax2.YTickLabel = {'EXTERNAL', 'RTK-GNSS'};


% ax1 = axes;
% ax2 = axes;
% axes(ax1);
% plot(1:1441,Output.results(:,32),':r',1:1441,data.results.state.signals.values(:,24),'--b')
% grid on
% set(legend('$SR_H^s(t)$ APT-simulator', '$SR_H^s(t)$ UVa/Padova T1DMS','Location','NorthWest'),'Interpreter','latex')
% set(ylabel('$SR_H^s(t)$ $[pmol/l/min]$'),'Interpreter','latex')
% set(gca,'XTick',[x_min:x_step:x_max],'XLim',[x_min x_max])
% axes(ax2);
% plot(1:1441,Output.results(:,3),':m',1:1441,data.results.state.signals.values(:,1),'--c')
% grid on
% set(ax2,'color','none','YAxisLocation','right');
% set(legend('$G(t)$ APT-simulator', '$G(t)$ UVa/Padova T1DMS','Location','NorthEast'),'Interpreter','latex')
% set(ylabel('$G(t)$ $[mg/dl]$'),'Interpreter','latex')
% set(gca,'XTick',[x_min:x_step:x_max],'XLim',[x_min x_max])