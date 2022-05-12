%% Map parameters
clear
res=10;
Ts = 0.05;

road_width = 50; % 50m*10
% road_length = 200; % 200m*10, urban
road_length = 200; % 100m*10, school zone

number_of_lanes = 3;
lane_width = 3; %3m*10

lane3 = road_width/2 - lane_width;
lane2 = road_width/2;
lane1 = road_width/2 + lane_width;

blank_map = zeros(number_of_lanes*lane_width*res, road_length*res); % *res for resolution. 0.1m per cell
road_edge = ones((road_width - number_of_lanes*lane_width)/2*res, road_length*res);
map = [road_edge; blank_map; road_edge];

%% Target Vehicle
% target1x = 100; % urban
target1x = 50;
target1y = lane3-0.5;
target1_length = 6; % m
target1_width = 2.3; % m

% target2x = 91;
target2x = 41;
target2y = lane3-0.5;
target2_length = 6; % m
target2_width = 2.3; % m


%% Child
child_length = 0.5; % 50cm
child_width = 0.5; % 50cm
child_v = 2; % m/s, + y direction in global

child_distance_to_target1 = 3; % m

childx = target1x + target1_length/2 + child_distance_to_target1;
childy = target1y - 2;

d_safe = 26.6; % 8.3m/s * 3.2s (vel * TTC)

%% Occupancy Map

map_show = occupancyMap(map,res);
map = flipud(map);

%% Draw on map
road_zero = road_width/2 - number_of_lanes*lane_width/2;
road_end=[0 road_length];
line1=[road_zero road_zero];
line2=[road_zero+lane_width road_zero+lane_width];
line3=[road_zero+lane_width*2 road_zero+lane_width*2];
line4=[road_zero+lane_width*3 road_zero+lane_width*3];

hold on
plot(road_end,line1, 'k-','LineWidth',2)
plot(road_end,line2, 'k--','LineWidth',2)
plot(road_end,line3, 'k--','LineWidth',2)
plot(road_end,line4, 'k-','LineWidth',2)

childplotx=[childx-child_length/2 childx+child_length/2];
childploty=[childy-child_width/2 childy+child_width/2];
plot([childplotx(1) childplotx(2)], [childploty(1) childploty(1)], 'r')
plot([childplotx(1) childplotx(2)], [childploty(2) childploty(2)], 'r')
plot([childplotx(1) childplotx(1)], [childploty(1) childploty(2)], 'r')
plot([childplotx(2) childplotx(2)], [childploty(1) childploty(2)], 'r')
text(childplotx(1)-1,childploty(1)-1,"Child")


target1plotx=[target1x-target1_length/2 target1x+target1_length/2];
target1ploty=[target1y-target1_width/2 target1y+target1_width/2];
plot([target1plotx(1) target1plotx(2)], [target1ploty(1) target1ploty(1)], 'm')
plot([target1plotx(1) target1plotx(2)], [target1ploty(2) target1ploty(2)], 'm')
plot([target1plotx(1) target1plotx(1)], [target1ploty(1) target1ploty(2)], 'm')
plot([target1plotx(2) target1plotx(2)], [target1ploty(1) target1ploty(2)], 'm')
text(target1plotx(1)+1,target1ploty(1)-1,"Target")

target2plotx=[target2x-target2_length/2 target2x+target2_length/2];
target2ploty=[target2y-target2_width/2 target2y+target2_width/2];
plot([target2plotx(1) target2plotx(2)], [target2ploty(1) target2ploty(1)], 'm')
plot([target2plotx(1) target2plotx(2)], [target2ploty(2) target2ploty(2)], 'm')
plot([target2plotx(1) target2plotx(1)], [target2ploty(1) target2ploty(2)], 'm')
plot([target2plotx(2) target2plotx(2)], [target2ploty(1) target2ploty(2)], 'm')
text(target2plotx(1)+1,target2ploty(1)-1,"Target")
hold off

% clear


%% Initialize

res=10;
TimeStep=0.1;

road_width = 50; % 50m*10
% road_length = 200; % urban
road_length = 200; % 100m*10
number_of_lanes = 3;
lane_width = 3; %3m*10

lane1 = road_width/2 - lane_width;
lane2 = road_width/2;
lane3 = road_width/2 + lane_width;

blank_map = zeros(number_of_lanes*lane_width*res, road_length*res); % *res for resolution. 0.1m per cell
road_edge = ones((road_width - number_of_lanes*lane_width)/2*res, road_length*res);
map = [road_edge; blank_map; road_edge];


% xlim([0,200])
xlim([0,100])
ylim([15,35])
title('Map')
% set(gcf, 'Position',  [100, 50, 1700, 170])
set(gcf, 'Position',  [100, 50, 1700, 340])
set(gca,'ytick',[])
