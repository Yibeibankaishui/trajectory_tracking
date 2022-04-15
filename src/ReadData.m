function [Latt,Long,Head] = ReadData()
%UNTITLED11 此处显示有关此函数的摘要
%   此处显示详细说明
data = readmatrix("/Users/yibeibankaishui/Desktop/轨迹跟踪和ros仿真/trajectory_tracking/data/GPCHC.csv");

Heading = data(:,4);
Lattitude = data(:,13);
Longitude = data(:,14);
GPSTime = data(:,3);
% 去除异常数据
Heading( Heading<-5 | Heading>360 ) = [];
Lattitude( Lattitude<30 | Lattitude>35) = [];
Longitude( Longitude<110 | Longitude>120) = [];
GPSTime( GPSTime<119773 ) = [];

Lattitude = 100000*(Lattitude - 21)-1102780;
Longitude = 100000*(Longitude - 118)-85620;
Time_ = GPSTime - GPSTime(1);

% 构造数据，1s为时间间隔
total = 888;
Latt = zeros(total,1);
Long = zeros(total,1);
Head = zeros(total,1);
Time = 0:total;
for i = 0:total-1
    inds = find(floor(Time_)==i);
    ind = inds(1);
    Latt(i+1) = Lattitude(ind);
    Long(i+1) = Longitude(ind);
    Head(i+1) = Heading(ind);
end

end

