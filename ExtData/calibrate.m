function [dataA_o,dataG_o] = calibrate(dataA, dataG, activity, nocandid)
% walking portion
st = 1;
ed = 6000;

accRS = dataA(st:end,1:3);
accRT = dataA(st:end,4:6);
accLS = dataA(st:ed,7:9);
accLT = dataA(st:ed,10:12);
accC = dataA(st:ed,13:15);

gyroRS = dataG(st:ed,1:3);
gyroRT = dataG(st:ed,4:6);
gyroLS = dataG(st:ed,7:9);
gyroLT = dataG(st:ed,10:12);
gyroC = dataG(st:ed,13:15);

activity = activity(st:ed);

%% find standing q_acc (tilt and roll)

stand = activity==0;

qacc_stand_RS = qacc(accRS(stand,:));
qacc_stand_RT = qacc(accRT(stand,:));
qacc_stand_LS = qacc(accLS(stand,:));
qacc_stand_LT = qacc(accLT(stand,:));
qacc_stand_C = qacc(accC(stand,:));
%% Transform data tilt and roll
aRS_g = accRS * quat2rotm(qacc_stand_RS');
gRS_g = gyroRS * quat2rotm(qacc_stand_RS');

aLS_g = accLS * quat2rotm(qacc_stand_LS');
gLS_g = gyroLS * quat2rotm(qacc_stand_LS');

aRT_g = accRT * quat2rotm(qacc_stand_RT');
gRT_g = gyroRT * quat2rotm(qacc_stand_RT');

aLT_g = accLT * quat2rotm(qacc_stand_LT');
gLT_g = gyroLT * quat2rotm(qacc_stand_LT');

aC_g = accC * quat2rotm(qacc_stand_C');
gC_g = gyroC * quat2rotm(qacc_stand_C');

walk = activity==1;
%% find walk yaw (rotate around global z-axis until rotation maxes)
%[angleC] = Attitude_Estimation3D(accC, gyroC, zeros(size(accC,1),1), 2, 2, 1, 0.17);

% if nocandid == 0 
%     yawRS = yawEstimate(aRS_g(:,:),gRS_g(:,:),angleC(:,:));%yawRS=0;
%     yawRT = yawEstimate(aRT_g(:,:),gRT_g(:,:),angleC(:,:));%yawRT=0;
%     yawLS = yawEstimate(aLS_g(:,:),gLS_g(:,:),angleC(:,:));% yawLS=0;
%     yawLT = yawEstimate(aLT_g(:,:),gLT_g(:,:),angleC(:,:));%yawLT=0;
%     yawC = yawCEstimate(aC_g(walk,:),angleC(walk,:));
% else
%     yawRS=0;yawLS=0;yawRT=0;yawLT=0;yawC=0;
% end
yawRS=0;yawLS=0;yawRT=0;yawLT=0;yawC=0;
% fprintf('RS Yaw: %3.0f deg\n',yawRS*180/pi);
% fprintf('RT Yaw: %3.0f deg\n',yawRT*180/pi);
% fprintf('LS Yaw: %3.0f deg\n',yawLS*180/pi);
% fprintf('LT Yaw: %3.0f deg\n',yawLT*180/pi);
% fprintf('C Yaw: %3.0f deg\n',yawC*180/pi);

%% Transform ouput results

accRS = dataA(:,1:3);
accRT = dataA(:,4:6);
accLS = dataA(:,7:9);
accLT = dataA(:,10:12);
accC = dataA(:,13:15);

gyroRS = dataG(:,1:3);
gyroRT = dataG(:,4:6);
gyroLS = dataG(:,7:9);
gyroLT = dataG(:,10:12);
gyroC = dataG(:,13:15);

aRS_o = accRS * quat2rotm(qacc_stand_RS') * [cos(yawRS) 0 sin(yawRS);0 1 0;-sin(yawRS) 0 cos(yawRS)];
gRS_o = gyroRS * quat2rotm(qacc_stand_RS') * [cos(yawRS) 0 sin(yawRS);0 1 0;-sin(yawRS) 0 cos(yawRS)];

aLS_o = accLS * quat2rotm(qacc_stand_LS') * [cos(yawLS) 0 sin(yawLS);0 1 0;-sin(yawLS) 0 cos(yawLS)];
gLS_o = gyroLS * quat2rotm(qacc_stand_LS') * [cos(yawLS) 0 sin(yawLS);0 1 0;-sin(yawLS) 0 cos(yawLS)];

aRT_o = accRT * quat2rotm(qacc_stand_RT') * [cos(yawRT) 0 sin(yawRT);0 1 0;-sin(yawRT) 0 cos(yawRT)];
gRT_o = gyroRT * quat2rotm(qacc_stand_RT') * [cos(yawRT) 0 sin(yawRT);0 1 0;-sin(yawRT) 0 cos(yawRT)];

aLT_o = accLT * quat2rotm(qacc_stand_LT') * [cos(yawLT) 0 sin(yawLT);0 1 0;-sin(yawLT) 0 cos(yawLT)];
gLT_o = gyroLT * quat2rotm(qacc_stand_LT') * [cos(yawLT) 0 sin(yawLT);0 1 0;-sin(yawLT) 0 cos(yawLT)];

aC_o = accC * quat2rotm(qacc_stand_C') * [cos(yawC) 0 sin(yawC);0 1 0;-sin(yawC) 0 cos(yawC)];
gC_o = gyroC * quat2rotm(qacc_stand_C') * [cos(yawC) 0 sin(yawC);0 1 0;-sin(yawC) 0 cos(yawC)];

dataA_o = [aRS_o,aRT_o,aLS_o,aLT_o,aC_o];
dataG_o = [gRS_o,gRT_o,gLS_o,gLT_o,gC_o];

% angleRS = Attitude_Estimation3D(aRS_o, gRS_o, angleC(:,2), 0.9, 3, 1, 0.17);
% angleLS = Attitude_Estimation3D(aLS_o, gLS_o, angleC(:,2), 0.9, 3, 1, 0.17);
% % 

%% Quaternion from Acceleration (Tilt and Roll) and Magnitude (Yaw to body gait FWD)
% qacc = [sqrt((ay+1)/2) az/sqrt(2*(ay+1)) 0 -ax/sqrt(2*(ay+1))];
% qacc2 = [-ax/sqrt(2*(1-ay)) 0 az/sqrt(2*(1-ay)) sqrt((1-ay)/2)];
% Ga = Ax^2 + Ay^2;
% qmag = [sqrt(Ga + Ax*sqrt(Ga))/sqrt(2*Ga) 0 -Az/sqrt(2*(Ga + Ax*sqrt(Ga))) 0 ];
% qmag2 = [Az/sqrt(2*(Ga - Ax*sqrt(Ga))) 0 -sqrt(Ga - Ax*sqrt(Ga))/sqrt(2*Ga) 0 ];
    function [qaccAVG] = qacc(a)
        ax = a(:,1);
        ay = a(:,2);
        az = a(:,3);
        qaccE = zeros(length(ax),4);  
        for ij = 1:length(ax)
            if ay(ij)>=0
                qaccE(ij,:) = [sqrt((ay(ij)+1)/2) az(ij)/sqrt(2*(ay(ij)+1)) 0 -ax(ij)/sqrt(2*(ay(ij)+1))];
            else
                qaccE(ij,:) = [-ax(ij)/sqrt(2*(1-ay(ij))) 0 az(ij)/sqrt(2*(1-ay(ij))) sqrt((1-ay(ij))/2)];
            end
        end
        qaccAVG = quatAvg(qaccE, ones(length(ax),1));
    end

%% Quaternion Avg
% by Tolga Birdal
% Q is an Mx4 matrix of quaternions. weights is an Mx1 vector, a weight for
% each quaternion.
% Qavg is the weightedaverage quaternion
% This function is especially useful for example when clustering poses
% after a matching process. In such cases a form of weighting per rotation
% is available (e.g. number of votes), which can guide the trust towards a
% specific pose. weights might then be interpreted as the vector of votes 
% per pose.
% Markley, F. Landis, Yang Cheng, John Lucas Crassidis, and Yaakov Oshman. 
% "Averaging quaternions." Journal of Guidance, Control, and Dynamics 30, 
% no. 4 (2007): 1193-1197.
    function [Qavg]=quatAvg(Q, weights)

        % Form the symmetric accumulator matrix
        A=zeros(4,4);
        M=size(Q,1);
        wSum = 0;

        for j=1:M
            q = Q(j,:)';
            w_i = weights(j);
            A=w_i.*(q*q')+A; % rank 1 update
            wSum = wSum + w_i;
        end

        % scale
        A=(1.0/wSum)*A;

        % Get the eigenvector corresponding to largest eigen value
        [Qavg, ~]=eigs(A,1);

    end
%% Find Yaw Angle (rotate around global z-axis to maximize pitch rate)
function [yawIMU]=yawEstimate(aRS_g,gRS_g,angleC)
    %tiltsRS = [];
    %yawsRS = [];
    yaw = (-90:5:90)*pi/180;
    X = zeros(1,length(yaw));
    
    for i = 1:length(yaw)
        aRS_y = aRS_g * [cos(yaw(i)) 0 -sin(yaw(i));0 1 0;sin(yaw(i)) 0 cos(yaw(i))];
        gRS_y = gRS_g * [cos(yaw(i)) 0 -sin(yaw(i));0 1 0;sin(yaw(i)) 0 cos(yaw(i))];
        angleRS = Attitude_Estimation3D(aRS_y, gRS_y, angleC(:,2), 0.9, 3, 1, 0.17, 120);
        % find maximum tilt
%         [pks,~] = findpeaks(angleRS(:,3),'MinPeakDistance',70,'MinPeakProminence',0.5);
%         [lks,~] = findpeaks(-angleRS(:,3),'MinPeakDistance',70,'MinPeakProminence',0.5);
%         X(i) = mean(pks) + mean(lks);
        maxRS = max(angleRS(:,3));
        minRS = min(angleRS(:,3));
        X(i) = maxRS - minRS;
    end
    
    % 2 Solutions! 1 FWD other 1 BWD direction
    [~, ind1RS] = max(X);
    %X(ind1RS)      = -Inf;
    
    % Try the first one:
    yawIMU = yaw(ind1RS);
    
    % output
    aRF_o = aRS_g * [cos(yawIMU) 0 sin(yawIMU);0 1 0;-sin(yawIMU) 0 cos(yawIMU)];
    gRF_o = gRS_g * [cos(yawIMU) 0 sin(yawIMU);0 1 0;-sin(yawIMU) 0 cos(yawIMU)];
    
    angleRS = Attitude_Estimation3D(aRF_o, gRF_o, angleC(:,2), 2, 2, 1, 0.17, 120);
    aRFi = Transform_acc3D(aRF_o, angleRS(:,3), angleRS(:,1));
    aRFi = aRFi - mean(aRFi);
    % Check if correlation of acc x and y axis match (should be>0)
    C1 = xcorr(aRFi(:,1),aRFi(:,2),'coeff');
%     
%     % Try the sec one:
%     yawIMU = yaw(ind1RS)+pi;
%     
%     % output
%     aRF_o = aRS_g * [cos(yawIMU) 0 sin(yawIMU);0 1 0;-sin(yawIMU) 0 cos(yawIMU)];
%     gRF_o = gRS_g * [cos(yawIMU) 0 sin(yawIMU);0 1 0;-sin(yawIMU) 0 cos(yawIMU)];
%     
%     angleRS = Attitude_Estimation3D(aRF_o, gRF_o, angleC(:,2), 2, 2, 1, 0.17);
%     aRFi = Transform_acc3D(aRF_o, angleRS(:,3), angleRS(:,1));
%     aRFi = aRFi - mean(aRFi);
%     % Check if correlation of acc x and y axis match (should be>0)
%     C2 = xcorr(aRFi(:,1),aRFi(:,2),'coeff');
%     figure
% %     t = 1:length(aRFi(:,1));
% %     plot(t,aRFi)
%     plot(C1)
    L = length(aRS_g(:,1));
    maxC1 = max(C1(L-20:L+20));
    minC1 = min(C1(L-20:L+20));
    
%     maxC2 = max(C2(4000:5999));
%     minC2 = min(C2(4000:5999));
%     
%     a1 = trapz(C1(5900:6100)*1000);
%     a2 = trapz(C2(5900:6100)*1000);
%    sprintf('First %f second %f',a1,a2)

    if abs(maxC1/minC1)>=0.8%(maxC2/minC2)
        yawIMU = yaw(ind1RS); % First answer is correct (FWD)
    else 
        yawIMU = yaw(ind1RS) + pi; % Second answer (+180 yaw) is correct (FWD)
    end
end

function [yawIMU]=yawCEstimate(aC_g,angleC)
        
    % Two Solutions!
    yawIMU1 = 0;
    yawIMU2 = pi;

    % C1 = xcorr(yawsRF(:,ind1RF),tiltsRF(:,ind1RF));
    % C2 = xcorr(yawsRF(:,ind2RF),tiltsRF(:,ind2RF));
    
    % Try the first one:
    aCi = Transform_acc3D(aC_g, angleC(:,3), angleC(:,1));
    
    % Check if correlation of x and y axis match (>0) 
    C1 = xcorr(aCi(:,1),aCi(:,2),'coeff');
    L = length(aCi(:,1));
    maxC = max(C1(L-500:L-5));
    minC = min(C1(L-500:L-5));
    
    if abs(maxC/minC)>=0.8
        yawIMU = yawIMU1; % First answer is correct
    else 
        yawIMU = yawIMU2; % Second answer (+180 yaw) is correct
    end
%     figure
%     plot(C1)
%     xlabel('Lag')
%     ylabel('Crosscorr')
%     
%     figure
%     t = 1:length(aCi(:,1));
%     plot(t,aCi)
end
end
