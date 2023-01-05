%% Localize with multi-point contact 
function e_posture(doPlot, saveVideo,dynamicsOn)

st = 1;
ed = 5000;
step = 1;
win = 100;
%saveVideo = 0;
%dynamicsOn = 0;
Fs = 120;
kp = 1000;
kd = 0;
ki = 0.001;
len = ed -st + 1;
% % Original data for retriving chest RG2B %%%%% U.RS, U.LS, U.SP, U.RFA, U.LFA
load('results/TestPredict.mat')
load('ExtData/PositionData.mat')
load('ExtData/nYQTestData.mat')
% Scales of the body dimensions
scale = [0.40;0.40;0.47;0.27;0.27;0.45;0.45;0.15;0.15;0.25;0.25;0.25;0.25];% RS LS SP RFA LFA RT LT RP LP RSH LSH RA LA [in meters]
%                                                                          %  1  2  3   4   5  6  7  8  9  10  11 12 13
%% reshape
Xtrue = Xact_t(st:ed,7:9);
vals = Y_t;
vals = vals(st:ed,:);
vals = reshape(vals,[],3,13);
vecs = permute(vals, [3 2 1]);
% 
vecs = Predict(st:ed,1:13,:);
vecs = permute(vecs, [2 3 1]);

RG2B = RG2B_t(:,:,st:ed);
%  for i=1:step:(length(RG2B_t)-win)
%      RG2B = cat(3,RG2B,RG2B_t(:,:,i+win));
%  end
% Rotate all vecs from Chest frame to Global frame
vecs = pagemtimes(vecs,'none',RG2B,'none');

%% Load acc data
if dynamicsOn
    load("ExtData/accTestData.mat");
    accRF = X_t(st:ed,1:3);
    accLF = X_t(st:ed,4:6);
    accSP = X_t(st:ed,7:9);
    accSP = lowpass(accSP, 0.5, 120);
    accSP = reshape(accSP',1,3,[]);
    accSP = pagemtimes(accSP,'none',RG2B,'none') + [0, 0, 9.8];
end
accSP = [0 0 0;0 0 0;diff(diff(Xtrue))*Fs^2];
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
    %sys = tf([1],[0.15 1]);
    %x(:,3) = lsim(sys,u,t);
%     figure(2)
%     plot( t, x(1:len,3) - (Xtrue(:,3)-0.43));
%     title('error')
%     figure(3)
%     plot( t, x(1:len,3), t,  x_g(:,3), t, (Xtrue(:,3)-0.43));
%     title('resp')    
%     figure(3)
%     plot( t, accSP(1:len,3), t, 20*(Xtrue(:,3)-0.43));
%     title('resp') 
    % keep only z for now
    x = [x_g(1:len,1:2), x(1:len,3)];

    % limit negative --> remove it when u enable physics
    x(x(1:len,3) < x_g(:,3), 3) = x_g(x(1:len,3) < x_g(:,3), 3);
    
    
end
%% plot

for i = 2:len
    
    
    % find lines
    l1 = cumsum([x(i,:);SP(:,:,i)*1.4]);
    l2 = cumsum([x(i,:);SP(:,:,i);RSH(:,:,i);RA(:,:,i);RFA(:,:,i)]);
    l3 = cumsum([x(i,:);SP(:,:,i);LSH(:,:,i);LA(:,:,i);LFA(:,:,i)]);
    l4 = cumsum([x(i,:);RP(:,:,i);RT(:,:,i);RS(:,:,i)]);
    l5 = cumsum([x(i,:);LP(:,:,i);LT(:,:,i);LS(:,:,i)]);
    
    % plot joints
    joints = [l4(4,:);l5(4,:);l2(2,:);l2(5,:);l3(5,:);
              l4(3,:);l5(3,:);l4(2,:);l5(2,:);
              l2(3,:);l3(3,:);l2(4,:);l3(4,:);
              l1(1,:);l1(2,:)];
    if doPlot
        set(handle(6),'MarkerSize',20);
        set(handle(1:5),'LineWidth',3);       
        set(handle(6), 'Xdata', joints(:,1), 'Ydata', joints(:,2), 'Zdata', joints(:,3));
     
        % plot lines
        set(handle(1), 'Xdata', l1(:,1), 'Ydata', l1(:,2), 'Zdata', l1(:,3));
        set(handle(2), 'Xdata', l2(:,1), 'Ydata', l2(:,2), 'Zdata', l2(:,3));
        set(handle(3), 'Xdata', l3(:,1), 'Ydata', l3(:,2), 'Zdata', l3(:,3));
        set(handle(4), 'Xdata', l4(:,1), 'Ydata', l4(:,2), 'Zdata', l4(:,3));
        set(handle(5), 'Xdata', l5(:,1), 'Ydata', l5(:,2), 'Zdata', l5(:,3));

        hold off
    end
    if saveVideo
        writeVideo(writerObj, getframe(gcf));
    end
    pause(frameLength);
end
if saveVideo
    close(writerObj);
end
end
