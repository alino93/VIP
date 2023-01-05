function [dataA,dataG,time1, activity] = AutoCalibrate(dataRS,dataRT,dataLS,dataLT,dataC,time1,timeS,timeE,Fs)
%% data handling
st=find(timeS-milliseconds(500)<=time1 & time1<=timeS+milliseconds(10000),1,'last');
ed=find(timeE-milliseconds(500)<=time1 & time1<=timeE+milliseconds(10000),1,'last');

accRS = dataRS(st:ed,2:4)/1000;
accRT = dataRT(st:ed,2:4)/1000;
accLS = dataLS(st:ed,2:4)/1000;
accLT = dataLT(st:ed,2:4)/1000;
accC = [dataC(st:ed,4) dataC(st:ed,3) -dataC(st:ed,2)]/1000;

gyroRS = dataRS(st:ed,5:7)*pi/180;
gyroRT = dataRT(st:ed,5:7)*pi/180;
gyroLS = dataLS(st:ed,5:7)*pi/180;
gyroLT = dataLT(st:ed,5:7)*pi/180;
gyroC = [dataC(st:ed,7) dataC(st:ed,6) -dataC(st:ed,5)]*pi/180;

time1 = time1(st:ed);

dataA = [accRS,accRT,accLS,accLT,accC];
dataG = [gyroRS,gyroRT,gyroLS,gyroLT,gyroC];

%% Walking detection
axRF = accRS(:,1); ayRF = accRS(:,2); azRF = accRS(:,3);
axLF = accLS(:,1); ayLF = accLS(:,2); azLF = accLS(:,3);

% filter acc
T=1/Fs;
t = (0:T:T*(length(axRF)-1));
armsR = zeros(length(t),1);
armsL = zeros(length(t),1);
L = length(t);
for i = 1:1:L-86
   armsR(i) = rms( [var(axRF(i:i+5)) , var(ayRF(i:i+5)), var(azRF(i:i+5))] );
   armsL(i) = rms( [var(axLF(i:i+5)) , var(ayLF(i:i+5)), var(azLF(i:i+5))] );
end

[~,Rlocs] = findpeaks(armsR,'MinPeakDistance',70/100*Fs);
[~,Llocs] = findpeaks(armsL,'MinPeakDistance',70/100*Fs);

activity = zeros(length(t),1);
%--------------------------------WALKING-----------------------------------
i = 1;
Dr = 30;
Dl = 30;
DSr=90;
DSl=90;
while i < (length(Rlocs))
    i = i + 1;
    if armsR(Rlocs(i))>1 
%         activity(Rlocs(i)+Dr,2) =12;
%         activity(Rlocs(i)+Dr-DSr,2)=11;
        activity(Rlocs(i)+Dr-DSr:Rlocs(i)+Dr+24,1) = ones(25+DSr,1)*1;
    end
end
i = 1;
while i < (length(Llocs))
    i = i + 1;
    if armsL(Llocs(i))>1
%         activity(Llocs(i)+Dl,3) =22;
%         activity(Llocs(i)+Dl-DSl,3)=21;
        activity(Llocs(i)+Dl-DSl:Llocs(i)+Dl+24,1) = ones(25+DSl,1)*1;
    end
end

%% check chest variabilitly (ckeck change of state bend/sit/lie) 
win = 6000; % 60 secs
stride = 1000; % 10 secs
overlap = win - stride;
Cstate = buffer(accC(:,1), win, overlap);
variab = var(Cstate,1);
% % 

% figure(5)
% plot(time1,activity)
% figure(6)
% plot(time1(inds(1,6:end)),variab(6:end))
% figure(7)
% plot(time1,armsR)
%% Find candidates of calibration
bins = buffer(activity(:,1), win, overlap);
candids = sum(bins,1);

% accepted walking ratio rate in a bin 
percent = 0.4; 

% index
ind = 1:1:length(axRF);
inds = buffer(ind, win, overlap);

cIdxss = find(candids < win * (1-0.1) & candids > win * percent & variab<0.008) ;%
cIdxss = [];
cIdxs = [];
if isempty(cIdxss)
    fprintf('No calibration candids!\n');
    [dataA(1:end,:), dataG(1:end,:)] = ...
        calibrate(dataA(1:end,:), dataG(1:end,:), activity(1:end,1), 1);
    return
end
    
% remove consecutive candidates
cIdxs = cIdxss(end);
for i=length(cIdxss):-1:2
    if cIdxs(1)-cIdxss(i-1)>5*10 % closer than 5 bins
        cIdxs=[cIdxss(i-1);cIdxs];
    end
end
cIdxss  = inds(1,cIdxss);
cIdxs = inds(1,cIdxs);
%% Auto-calibrate 

for i=1:length(cIdxs)-1
    fprintf('Time of calibration: %s\n',time1(cIdxs(i)));
    [dataA(cIdxs(i)+1:cIdxs(i+1),:), dataG(cIdxs(i)+1:cIdxs(i+1),:)] = ...
        calibrate(dataA(cIdxs(i)+1:cIdxs(i+1),:), dataG(cIdxs(i)+1:cIdxs(i+1),:), activity(cIdxs(i)+1:cIdxs(i+1),1), 0);
end
fprintf('Time of calibration: %s\n',time1(cIdxs(end)));
[dataA(cIdxs(end)+1:end,:), dataG(cIdxs(end)+1:end,:)] = ...
        calibrate(dataA(cIdxs(end)+1:end,:), dataG(cIdxs(end)+1:end,:), activity(cIdxs(end)+1:end,1), 0);
