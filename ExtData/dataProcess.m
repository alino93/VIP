function [data1, data2, data3, data4, data5, time1] = dataProcess(nameRS,nameLS,nameRT,nameLT,nameC,freq)
T = 1/freq;

d1 = readtable(nameRS); 
data1 = d1{:,3:8};
time1 = d1{:,2};
d1 = readtable(nameLS);
data2 = d1{:,3:8};
time2 = d1{:,2};
d1 = readtable(nameRT);
data3 = d1{:,3:8};
time3 = d1{:,2};
d1 = readtable(nameLT);
data4 = d1{:,3:8};
time4 = d1{:,2};
d1 = readtable(nameC);
data5 = d1{:,3:8};
time5 = d1{:,2};

%% crop 
t0 = max([time1(1),time2(1),time3(1),time4(1),time5(1)]);
 
t1 = milliseconds(time1-t0)/1000;
data1 = [t1 data1];
t2 = milliseconds(time2-t0)/1000;
data2 = [t2 data2];
t3 = milliseconds(time3-t0)/1000;
data3 = [t3 data3];
t4 = milliseconds(time4-t0)/1000;
data4 = [t4 data4];
t5 = milliseconds(time5-t0)/1000;
data5 = [t5 data5];

tf = min([t1(end),t2(end),t3(end),t4(end),t5(end)]);

%% interpolate for lost data 
t = (0:T:tf)';
[t1, index] = unique(t1); 
data1 = interp1(t1,data1(index,:),t);
time1 = interp1(t1,time1(index,:),t);
[t2, index] = unique(t2); 
data2 = interp1(t2,data2(index,:),t);
[t3, index] = unique(t3); 
data3 = interp1(t3,data3(index,:),t);
[t4, index] = unique(t4); 
data4 = interp1(t4,data4(index,:),t);
[t5, index] = unique(t5); 
data5 = interp1(t5,data5(index,:),t);


%% sync 2 by 2 (all with C)

% make lengths the same
len = min([length(data1(:,1)),length(data2(:,1)),...
    length(data3(:,1)),length(data4(:,1)),length(data5(:,1))]);

data1 = data1(1:len,:);
data2 = data2(1:len,:);
data3 = data3(1:len,:);
data4 = data4(1:len,:);
data5 = data5(1:len,:);
time1 = time1(1:len,:);

%% Rotate signals 
% default sign of LS and LT sensors
data2(:,2) = data2(:,2)*-1;
data2(:,4) = data2(:,4)*-1;
data4(:,2) = data4(:,2)*-1;
data4(:,4) = data4(:,4)*-1;

data2(:,5) = data2(:,5)*-1;
data2(:,7) = data2(:,7)*-1;
data4(:,5) = data4(:,5)*-1;
data4(:,7) = data4(:,7)*-1;
%data5 = [t, data5(:,4), data5(:,3), -data5(:,2), data5(:,7), data5(:,6), -data5(:,5)];


end
% %% sync function
% function [data1,data2] = sync(data1,data2,calibT,time)
% %gyros
% tilt1Rate = lowpass(data1(calibT-5e3:calibT+5e3,6),4,100)*  pi/180;
% tilt1Rate = tilt1Rate - mean(tilt1Rate(1:500));
% %gyros
% tilt2Rate = lowpass(data2(calibT-5e3:calibT+5e3,6),4,100)*  pi/180;
% tilt2Rate = tilt2Rate - mean(tilt2Rate(1:500));
% %correlation
% [C1,lag1] = xcorr(tilt1Rate,tilt2Rate);
% [~,I] = max(abs(C1));
% SampleDiff = lag1(I)
% if SampleDiff>0
%     data2 = [zeros(SampleDiff,10); data2];
% else
%     data2 = data2(-SampleDiff+1:end,:);
% end
% 
% end