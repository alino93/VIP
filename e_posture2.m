%% Localize with multi-point contact 
function e_posture2(doPlot, saveVideo, dynamicsOn)
st = 1;
ed = 5000;
step = 1;
win = 100;
% saveVideo = 0;
% dynamicsOn = 1;
% doPlot = 0;
Fs = 120;
kp = 1000;
kd = 0;
ki = 0.001;
len = ed -st + 1;
% % Original data for retriving chest RG2B %%%%% U.RS, U.LS, U.SP, U.RFA, U.LFA
load('ExtData/nYQTestData.mat')
load('ExtData/PositionData.mat')

% Scales of the body dimensions
scale = [0.40;0.40;0.47;0.27;0.27;0.45;0.45;0.15;0.15;0.25;0.25;0.25;0.25];% RS LS SP RFA LFA RT LT RP LP RSH LSH RA LA [in meters]
%                                                                          %  1  2  3   4   5  6  7  8  9  10  11 12 13
%% reshape
%Xtrue = Xact_t(st:ed,7:9);
vals = Y_t;
vals = vals(st:ed,:);
vals = reshape(vals,[],3,13);
vecs = permute(vals, [3 2 1]);

RG2B = RG2B_t(:,:,st:ed);
%  for i=1:step:(length(RG2B_t)-win)
%      RG2B = cat(3,RG2B,RG2B_t(:,:,i+win));
%  end
% Rotate all vecs from Chest frame to Global frame
vecs = pagemtimes(vecs,'none',RG2B,'none');

%% Load acc data
%%%%% X = ALLA = A.RF A.LF A.RFA A.LFA A.C
%%%%% Y = AllU = U.RS, U.LS, U.RT, U.LT, U.RP, U.LP, U.SP, U.RSH, U.LSH, U.RA, U.LA, U.RFA, U.LFA
load("ExtData/accTestData.mat");
accRF = X_t(st:ed,1:3);
accLF = X_t(st:ed,4:6);
accSP = X_t(st:ed,13:15);

% transform local total acc to global trans/linear acc
% method 1
%accSP = lowpass(accSP, 0.5, 120);
accSP = reshape(accSP',1,3,[]);
accSP = pagemtimes(accSP,'none',RG2B,'none') - [0, 0, 9.81];
accSP = reshape(accSP,3,[])';

% method 2
%accSP2 = [0 0 0;0 0 0;diff(diff(Xtrue))*Fs^2];

% figure(2)
% plot(accSP(:,1))
% hold on
% plot(accSP2(:,1))
%%
% change order of vecs and scale to true sizes
% RS LS SP RFA LFA RT LT RP LP RSH LSH RA LA [in meters]
%  1  2  3   4   5  6  7  8  9  10  11 12 13
%U.RS, U.LS, U.RT, U.LT, U.RP, U.LP, U.SP, U.RSH, U.LSH, U.RA, U.LA, U.RFA, U.LFA
%   1     2     3     4     5     6     7      8      9    10    11     12     13
id2idx = [1 2 7 12 13 3 4 5 6 8 9 10 11];
vecsTemp = vecs;
vecs = vecsTemp(id2idx,:,:);

RS = vecs(1,:,:)*scale(1);
LS = vecs(2,:,:)*scale(2);
SP = vecs(3,:,:)*scale(3);
RFA = vecs(4,:,:)*scale(4);
LFA = vecs(5,:,:)*scale(5);

RT = vecs(6,:,:)*scale(6);
LT = vecs(7,:,:)*scale(7); 
RP = vecs(8,:,:)*scale(8); 
LP = vecs(9,:,:)*scale(9);

RSH = vecs(10,:,:)*scale(10);
LSH = vecs(11,:,:)*scale(11);
RA = vecs(12,:,:)*scale(12);
LA = vecs(13,:,:)*scale(13);

%% Initialize
i = 1; % step
x_g = zeros (len,3);
v_org = [0 0 0];
frameLength = 1/Fs*step;

l1 = cumsum([x_g(i,:);SP(:,:,i)*1.4]);
l2 = cumsum([x_g(i,:);SP(:,:,i);RSH(:,:,i);RA(:,:,i);RFA(:,:,i)]);
l3 = cumsum([x_g(i,:);SP(:,:,i);LSH(:,:,i);LA(:,:,i);LFA(:,:,i)]);
l4 = cumsum([x_g(i,:);RP(:,:,i);RT(:,:,i);RS(:,:,i)]);
l5 = cumsum([x_g(i,:);LP(:,:,i);LT(:,:,i);LS(:,:,i)]);

% Joints cheatsheet
% RS LS SP RFA LFA RT LT RP LP RSH LSH RA LA Org Head [in meters]
%  1  2  3   4   5  6  7  8  9  10  11 12 13  14   15
joints = [l4(4,:);l5(4,:);l2(2,:);l2(5,:);l3(5,:);
          l4(3,:);l5(3,:);l4(2,:);l5(2,:);
          l2(3,:);l3(3,:);l2(4,:);l3(4,:);
          l1(1,:);l1(2,:)];

% make initial foot height zero
joints = joints - mean([joints(1,:);joints(2,:)]);
x_g(i,:) = joints(14,:);
l1 = cumsum([x_g(i,:);SP(:,:,i)*1.4]);
l2 = cumsum([x_g(i,:);SP(:,:,i);RSH(:,:,i);RA(:,:,i);RFA(:,:,i)]);
l3 = cumsum([x_g(i,:);SP(:,:,i);LSH(:,:,i);LA(:,:,i);LFA(:,:,i)]);
l4 = cumsum([x_g(i,:);RP(:,:,i);RT(:,:,i);RS(:,:,i)]);
l5 = cumsum([x_g(i,:);LP(:,:,i);LT(:,:,i);LS(:,:,i)]);

% which joint is in ZUPT?
% joints with z (height) < 1 cm
ZUPTS = find(joints(1:13,3)<0.01);
% paths to org
path = {[1,6,8];
        [2,7,9];
        [3];
        [4,12,10,3];
        [5,13,11,3];
        [6,8];
        [7,9];
        [8];
        [9];
        [10,3];
        [11,3];
        [12,10,3];
        [13,11,3];};

% Force ZUPT
new_orgs = 0;

for ii = 1:length(ZUPTS)
    % the jount in zupt
    jt = ZUPTS(ii);
    % vect from that jt to org
    new_orgs = new_orgs + ...
    joints(jt,:) - sum(vecs(path{jt},:,i) .* scale(path{jt}), 1);
end

% if no contact point, continue with previous org velocity
if isempty(ZUPTS)
    x_g = x_g + v_org;
    v_org = v_org - [0 0 9.8] / Fs;
else
    v_org = x_g - new_orgs / length(ZUPTS);
    x_g = new_orgs / length(ZUPTS);
end

xlim = [-2,2.5];
ylim = [-2,2.5];
zlim = [-0.1,2];

if doPlot
    figure(1)
    clf;
    hold on
    grid on
    axis on
    axis equal
    view(45,30)

    handle(1) = line(l1(:,1),l1(:,2),l1(:,3));
    handle(2) = line(l2(:,1),l2(:,2),l2(:,3));
    handle(3) = line(l3(:,1),l3(:,2),l3(:,3));
    handle(4) = line(l4(:,1),l4(:,2),l4(:,3));
    handle(5) = line(l5(:,1),l5(:,2),l5(:,3));
    handle(6) = plot3(joints(:,1), joints(:,2), joints(:,3), 'r.');

    set(handle(1), 'markersize', 20);
    set(handle(1), 'MarkerFaceColor',[1,0,0])

    xlabel('x')
    ylabel('z')
    zlabel('y')

    set(gca, 'xlim', xlim, 'ylim', ylim, 'zlim', zlim,'LineWidth',3,...
        'XTickLabel',[],'YTickLabel',[],'ZTickLabel',[],'XLabel',[],'YLabel',[],'ZLabel',[]);
end
if saveVideo
    writerObj = VideoWriter(saveVideoName);
    writerObj.FrameRate = Fs;
    open(writerObj);
end
%%
% Update ZUPT and Geo Estimation

for i = 2:len
    
    % which joint is in ZUPT?
    % joints with z (height) < 1 cm
    ZUPTS = find(joints(1:13,3)<0.01);
    
    % Force ZUPT
    new_orgs = 0;

    for ii = 1:length(ZUPTS)
        % the jount in zupt
        jt = ZUPTS(ii);
        % vect from that jt to org
        new_orgs = new_orgs + ...
        [joints(jt,1:2),0] - sum(vecs(path{jt},:,i) .* scale(path{jt}), 1);
    end

    % if no contact point, continue with previous org velocity
    if isempty(ZUPTS)
        x_g(i,:) = x_g(i-1,:) + v_org;
        %v_org = v_org - [0 0 9.8] / Fs;
    else
        v_org = new_orgs / length(ZUPTS) - x_g(i-1,:);
        x_g(i,:) = new_orgs / length(ZUPTS);
    end
    
    
    % find lines
    l1 = cumsum([x_g(i,:);SP(:,:,i)*1.4]);
    l2 = cumsum([x_g(i,:);SP(:,:,i);RSH(:,:,i);RA(:,:,i);RFA(:,:,i)]);
    l3 = cumsum([x_g(i,:);SP(:,:,i);LSH(:,:,i);LA(:,:,i);LFA(:,:,i)]);
    l4 = cumsum([x_g(i,:);RP(:,:,i);RT(:,:,i);RS(:,:,i)]);
    l5 = cumsum([x_g(i,:);LP(:,:,i);LT(:,:,i);LS(:,:,i)]);
    
    % plot joints
    joints = [l4(4,:);l5(4,:);l2(2,:);l2(5,:);l3(5,:);
              l4(3,:);l5(3,:);l4(2,:);l5(2,:);
              l2(3,:);l3(3,:);l2(4,:);l3(4,:);
              l1(1,:);l1(2,:)];

end


%% dynamics
% dynamic root/org position
x = x_g;

if dynamicsOn
    accSPP = accSP;
    %accSPP(accSP(:,3)>0,3) = accSP(accSP(:,3)>0,3) ;
    kp = -0.020;%0.009
    kd = 0.29;%0.09
    ki = 0.000005;%0
    for i = 2:len

        a = accSPP(i,:) + (kp * (x_g(i,:) - x(i,:)) + ...
            kd * Fs * (x_g(i,:) - x_g(i-1,:) - x(i,:) + x(i-1,:)) + ...
            ki * trapz(x_g(1:i,:) - x(1:i,:))) ;
        x(i+1,:) = a / Fs^2 + 2 * x(i,:) - x(i-1,:);

    end
    
    t = 0:1/Fs:(ed-st)/Fs;

    % keep only z for now
    x = [x_g(1:len,1:2), x(1:len,3)];

    % limit negative --> remove it when u enable physics
    x(x(1:len,3) < x_g(:,3), 3) = x_g(x(1:len,3) < x_g(:,3), 3);
    
    
end
%% plot
joints = zeros(15,3,len);

for i = 2:len
         
    % find lines
    l1 = cumsum([x(i,:);SP(:,:,i)*1.4]);
    l2 = cumsum([x(i,:);SP(:,:,i);RSH(:,:,i);RA(:,:,i);RFA(:,:,i)]);
    l3 = cumsum([x(i,:);SP(:,:,i);LSH(:,:,i);LA(:,:,i);LFA(:,:,i)]);
    l4 = cumsum([x(i,:);RP(:,:,i);RT(:,:,i);RS(:,:,i)]);
    l5 = cumsum([x(i,:);LP(:,:,i);LT(:,:,i);LS(:,:,i)]);
    
    % plot joints
    joints(:,:,i) = [l4(4,:);l5(4,:);l2(2,:);l2(5,:);l3(5,:);
              l4(3,:);l5(3,:);l4(2,:);l5(2,:);
              l2(3,:);l3(3,:);l2(4,:);l3(4,:);
              l1(1,:);l1(2,:)];
          
    if doPlot
        set(handle(6),'MarkerSize',20);
        set(handle(1:5),'LineWidth',3);  
        
        set(handle(6), 'Xdata', joints(:,1,i), 'Ydata', joints(:,2,i), 'Zdata', joints(:,3,i));

        % plot lines
        set(handle(1), 'Xdata', l1(:,1), 'Ydata', l1(:,2), 'Zdata', l1(:,3));
        set(handle(2), 'Xdata', l2(:,1), 'Ydata', l2(:,2), 'Zdata', l2(:,3));
        set(handle(3), 'Xdata', l3(:,1), 'Ydata', l3(:,2), 'Zdata', l3(:,3));
        set(handle(4), 'Xdata', l4(:,1), 'Ydata', l4(:,2), 'Zdata', l4(:,3));
        set(handle(5), 'Xdata', l5(:,1), 'Ydata', l5(:,2), 'Zdata', l5(:,3));

        hold off
    
        if saveVideo
            writeVideo(writerObj, getframe(gcf));
        end
        pause(frameLength);
    end
end
if saveVideo
    close(writerObj);
end


%% Save data for optimization
load('ExtData/True_Q_X_Test.mat','Q_t','X_t')

%X_t = reshape(X_t,[],3,13);
Q_t = reshape(Q_t,[],4,13);


% p: pose/orientation in local root frame  n x 24 x 3 x 3
% v: joint velocity in local root frame n x 24 x 3
% c: binary contact of foot joints n x 2 (LF, RF)
% a: global acceleration of measured joints  n x 6 x 3
p = zeros(len,24,3,3);
v = zeros(len,24,3);
c = zeros(len,2);
a = zeros(len,6,3);
% 24 joints : ['ROOT', 'LHIP', 'RHIP', 'SPINE1', 'LKNEE', 'RKNEE', 'SPINE2', 'LANKLE', 'RANKLE',
%              'SPINE3', 'LFOOT', 'RFOOT', 'NECK', 'LCLAVICLE', 'RCLAVICLE', 'HEAD', 'LSHOULDER',
%              'RSHOULDER', 'LELBOW', 'RELBOW', 'LWRIST', 'RWRIST', 'LHAND', 'RHAND']
frame = 1;
jt = 1;
% Joints cheatsheet
% RS LS SP RFA LFA RT LT RP LP RSH LSH RA LA Org Head [in meters]
%  1  2  3   4   5  6  7  8  9  10  11 12 13  14   15
% map eskeleton vec to SMPL joints
% SMPL joint to my vectors
jt2ve = [3 7 6 3 2 1 3 2 1,...
         3 2 1 3 11 10 3 13,...
         12 5 4 5 4 5 4];
% SMPL joint to my relative neighbor vectors
jt2neiVe = [0 3 3 3 7 6 3 2 1,...
            3 2 1 3 3 3 3 11,...
            10 13 12 5 4 5 4];
% SMPL joint initial euler rotation in XYZ frame (jt2in)
jt2inEul = zeros(13,3);
jt2inEul([13,12],:) = [90 0 0;-90 0 0]*-pi/180;
% My frame to SMPL frame
Rm2smpl = [0 0 -1;0 1 0;1 0 0] * [1 0 0;0 0 1;0 -1 0];


% SMPL joint to my joints
jt2j = [14 9 8 3 7 6 3 2 1 3 2 1 15 11 10 15 11 10 13 12 5 4 5 4];
angle = zeros(1,3);
for frame=2:len
    for jt = 1:24
%         vector = vecs(jt2ve(jt),:,frame);
%         angle(3) = 0;
%         angle(2) = asin(vector(1));
%         angle(1) = atan2(-vector(2),vector(3));

        % pose/ori of joint in global (jt 2 global) * initial pose of some joints (g2jt * jt2in)
        rotm_jt = quat2dcm(Q_t(frame,:,jt2ve(jt)));
        
        % ori of neighbor joint in global (njt 2 global) * initial pose of some joints (g2jt * jt2in)
        if jt==1
            rotm_njt = eye(3);
        else
            rotm_njt = quat2dcm(Q_t(frame,:,jt2neiVe(jt))) ;%RG2B(:,:,frame) * rotm; 
        end

        % pose of jt relative to njoint (jt2njt = g2njt * initial_ori of jts * jt2g )
        rotm =  rotm_njt' * eul2rotm(jt2inEul(jt2ve(jt),:),'XYZ') * rotm_jt;%
        
        % correction of elbow joint frame mismatch
        if jt==20 
           temp = rotm2quat(rotm);
           rotm = quat2rotm([temp(1:2) temp(4) -temp(3)]);
        end
        if jt==19
           temp = rotm2quat(rotm);
           rotm = quat2rotm([temp(1:2) -temp(4) temp(3)]);
        end

        % pose of each joint wrt neig jt in XYZ (Rm2smpl transforms XYZ to ZYX) 
        temp = Rm2smpl * rotm;
        p(frame,jt,:,:) = [temp(:,2:3), temp(:,1)]; % reorder columns ZXY to XYZ
        % velocity arracy in global
        vel = (joints(jt2j(jt),:,frame) - joints(jt2j(jt),:,frame-1))*Fs;
        % velocity arracy in local root
        %vel = RG2B(:,:,frame) * vel'; 
        % velocity arracy in global frame
        v(frame,jt,:) = vel;
    end
    % contact logical array frames x [LF RF] each row
    c(frame,:) = joints([2,1],3,frame)<0.01;
    
end
accRF = [0 0 0;0 0 0;diff(diff(Xact_t(st:ed,1:3)))*Fs^2];
accLF = [0 0 0;0 0 0;diff(diff(Xact_t(st:ed,4:6)))*Fs^2];
accSP = [0 0 0;0 0 0;diff(diff(Xact_t(st:ed,7:9)))*Fs^2];
accRFA = [0 0 0;0 0 0;diff(diff(Xact_t(st:ed,10:12)))*Fs^2];
accLFA = [0 0 0;0 0 0;diff(diff(Xact_t(st:ed,13:15)))*Fs^2];
% SMPL acc order: ['LWRIST', 'RWRIST', 'LKNEE', 'RKNEE', 'HEAD', 'ROOT']
a(:,1,:) = accLFA;
a(:,2,:) = accRFA;
a(:,3,:) = accLF;
a(:,4,:) = accRF;
a(:,5,:) = accSP;
a(:,6,:) = accSP;

% switch axes X Y Z -> Z X Y
temp = a(:,:,3);
a(:,:,3) = a(:,:,1);
a(:,:,1) = a(:,:,2);
a(:,:,2) = temp;


temp = v(:,:,3);
v(:,:,3) = v(:,:,1);
v(:,:,1) = v(:,:,2);
v(:,:,2) = temp;

% temp = p(:,:,3);
% p(:,:,3) = p(:,:,2);
% p(:,:,2) = p(:,:,1);
% p(:,:,1) = temp;

save('data/DataForOptz.mat','a','v','p','c');


%% Save data for optimization %%%%%% USING VECTORS
load('ExtData/nYQTestData.mat')
%load('ExtData/PositionData.mat')
vals = Y_t;
vals = vals(st:ed,:);
vals = reshape(vals,[],3,13);
vecs = permute(vals, [3 2 1]);
% change order of vecs and scale to true sizes
% RS LS SP RFA LFA RT LT RP LP RSH LSH RA LA [in meters]
%  1  2  3   4   5  6  7  8  9  10  11 12 13
%U.RS, U.LS, U.RT, U.LT, U.RP, U.LP, U.SP, U.RSH, U.LSH, U.RA, U.LA, U.RFA, U.LFA
%   1     2     3     4     5     6     7      8      9    10    11     12     13
id2idx = [1 2 7 12 13 3 4 5 6 8 9 10 11];
vecsTemp = vecs;
vecs = vecsTemp(id2idx,:,:);

% p: pose/orientation in local root frame  n x 24 x 3 x 3
% v: joint velocity in local root frame n x 24 x 3
% c: binary contact of foot joints n x 2 (LF, RF)
% a: global acceleration of measured joints  n x 6 x 3
p = zeros(len,24,3,3);
v = zeros(len,24,3);
c = zeros(len,2);
a = zeros(len,6,3);
% 24 joints : ['ROOT', 'LHIP', 'RHIP', 'SPINE1', 'LKNEE', 'RKNEE', 'SPINE2', 'LANKLE', 'RANKLE',
%              'SPINE3', 'LFOOT', 'RFOOT', 'NECK', 'LCLAVICLE', 'RCLAVICLE', 'HEAD', 'LSHOULDER',
%              'RSHOULDER', 'LELBOW', 'RELBOW', 'LWRIST', 'RWRIST', 'LHAND', 'RHAND']
frame = 1;
jt = 1;
% Joints cheatsheet
% RS LS SP RFA LFA RT LT RP LP RSH LSH RA LA Org Head [in meters]
%  1  2  3   4   5  6  7  8  9  10  11 12 13  14   15
% map eskeleton vec to SMPL joints
% SMPL joint to my vectors
jt2ve = [3 7 6 3 2 1 3 2 1,...
         3 2 1 3 11 10 3 13,...
         12 5 4 5 4 5 4];
% SMPL joint to my relative neighbor vectors
jt2neiVe = [0 3 3 3 7 6 3 2 1,...
            3 2 1 3 3 3 3 11,...
            10 13 12 5 4 5 4];
% SMPL joint initial euler rotation in XYZ frame (jt2in)
jt2inEul = zeros(13,3);
jt2inEul([13,12],:) = [90 0 0;-90 0 0]*-pi/180;
% My frame to SMPL frame
Rm2smpl = [0 0 -1;0 1 0;1 0 0] * [1 0 0;0 0 1;0 -1 0];

% Correction on vecs direction (all vecs upward)
vecsC = -vecs;
vecsC([3],:,:) = vecs([3],:,:);

% SMPL joint to my joints
jt2j = [14 9 8 3 7 6 3 2 1 3 2 1 15 11 10 15 11 10 13 12 5 4 5 4];
angle = zeros(1,3);
for frame=2:len
    for jt = 1:24
        
        % ori of neighbor joint in global (njt 2 global) * initial pose of some joints (g2jt * jt2in)
%         if jt2ve(jt)==3
%             rotm_jt = RG2B(:,:,frame);
%         else
            vector = vecsC(jt2ve(jt),:,frame);
            angle(3) = 0;
            angle(1) = atan2(-vector(2),vector(3));
            angle(2) = atan2(vector(1),vector(3)/cos(angle(1)));
            % pose/ori of joint in global (jt 2 global) * initial pose of some joints (g2jt * jt2in)
            rotm_jt = eul2rotm(angle,'XYZ');
%         end
        if jt==1
            rotm_njt = eye(3);
            rotm_jt = RG2B(:,:,frame)';%* eul2rotm([0 10 0]*pi/180,'XYZ')
%         elseif jt2neiVe(jt)==3
%             rotm_njt = RG2B(:,:,frame);
        else
            vector = vecsC(jt2neiVe(jt),:,frame);
            angle(3) = 0;
            %angle(2) = -asin(vector(1));
            angle(1) = atan2(-vector(2),vector(3));
            angle(2) = atan2(vector(1),vector(3)/cos(angle(1)));
            rotm_njt = eul2rotm(angle,'XYZ');%RG2B(:,:,frame) * rotm; 
        end

        % pose of jt relative to njoint (jt2njt = g2njt * initial_ori of jts * jt2g )
        rotm =  rotm_njt' * eul2rotm(jt2inEul(jt2ve(jt),:),'XYZ') * rotm_jt;%
        
        % correction of hip joint yaw inconsistency 
%         if jt==2
%            rotm = rotm * eul2rotm([0.1 0 -0.1],'XYZ');
%         end
%         if jt==3
%            rotm = rotm * eul2rotm([0.1 0 0.1],'XYZ');
%         end
        
        % correction of arm and elbow joint frame mismatch
        if jt==20 || jt == 18
           temp = rotm2quat(rotm);
           rotm = quat2rotm([temp(1:2) temp(4) -temp(3)]);
        end
        if jt==19 || jt == 17
           temp = rotm2quat(rotm);
           rotm = quat2rotm([temp(1:2) -temp(4) temp(3)]);
        end

        % pose of each joint wrt neig jt in XYZ (Rm2smpl transforms XYZ to ZYX) 
        temp = Rm2smpl * rotm;
        p(frame,jt,:,:) = [temp(:,2:3), temp(:,1)]; % reorder columns ZXY to XYZ
        % velocity arracy in global
        vel = (joints(jt2j(jt),:,frame) - joints(jt2j(jt),:,frame-1))*Fs;
        % velocity arracy in global frame
        v(frame,jt,:) = vel;
    end
    % contact logical array frames x [LF RF] each row
    c(frame,:) = joints([2,1],3,frame)<0.01;
    
end
accRF = [0 0 0;0 0 0;diff(diff(Xact_t(st:ed,1:3)))*Fs^2];
accLF = [0 0 0;0 0 0;diff(diff(Xact_t(st:ed,4:6)))*Fs^2];
accSP = [0 0 0;0 0 0;diff(diff(Xact_t(st:ed,7:9)))*Fs^2];
accRFA = [0 0 0;0 0 0;diff(diff(Xact_t(st:ed,10:12)))*Fs^2];
accLFA = [0 0 0;0 0 0;diff(diff(Xact_t(st:ed,13:15)))*Fs^2];
% SMPL acc order: ['LWRIST', 'RWRIST', 'LKNEE', 'RKNEE', 'HEAD', 'ROOT']
a(:,1,:) = accLFA;
a(:,2,:) = accRFA;
a(:,3,:) = accLF;
a(:,4,:) = accRF;
a(:,5,:) = accSP;
a(:,6,:) = accSP;

% switch axes X Y Z -> Z X Y
temp = a(:,:,3);
a(:,:,3) = a(:,:,1);
a(:,:,1) = a(:,:,2);
a(:,:,2) = temp;


temp = v(:,:,3);
v(:,:,3) = v(:,:,1);
v(:,:,1) = v(:,:,2);
v(:,:,2) = temp;

save('data/DataForOptzV.mat','a','v','p','c');
end
