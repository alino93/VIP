%% Load and Sync IMU Data
function ExtractIMUData()

nameRF='IMU/RF.TXT';
nameLF='IMU/LF.TXT';
nameRH='IMU/RH.TXT';
nameLH='IMU/LH.TXT';
nameC='IMU/C.TXT';

Fs = 120;
g = 9.81;
[dataRF, dataLF, dataRH, dataLH, dataC, time1] = dataProcess(nameRF,nameLF,nameRH,nameLH,nameC,Fs);

timeS = duration(18,36,10);
timeE = duration(18,38,03);

[dataA,dataG,time1, activity] = AutoCalibrate(dataRF,dataRH,dataLF,dataLH,dataC,time1,timeS,timeE,Fs);

% shape: 1 (FWD),2 (UP),3 (RIGHT/EAST) 
accRF = dataA(:,1:3)*g;
accRH = dataA(:,4:6)*g;
accLF = dataA(:,7:9)*g;
accLH = dataA(:,10:12)*g;
accC = dataA(:,13:15)*g;

%gyros
% remove noise and bias
dataG = lowpass(dataG,10,Fs);
gyroRF = dataG(:,1:3)-mean(dataG(1:250,1:3));
gyroRH = dataG(:,4:6)-mean(dataG(1:250,4:6));
gyroLF = dataG(:,7:9)-mean(dataG(1:250,7:9));
gyroLH = dataG(:,10:12)-mean(dataG(1:250,10:12));
gyroC = dataG(:,13:15)-mean(dataG(1:250,13:15));


time = (0:1:size(accC,1)-1)/Fs;

%% Change data to required shape for ML
%                     |__A : total acceleration (with gravity) [m/s^2]
%                        |__RF, LF, RFA, LFA, C
%                            |__ x(FWD/NORTH) y(LAT/WEST) z(VERT/UP)_NWU
accRF = [accRF(:,1), -accRF(:,3), accRF(:,2)];
accRH = [accRH(:,1), -accRH(:,3), accRH(:,2)];
accLF = [accLF(:,1), -accLF(:,3), accLF(:,2)];
accLH = [accLH(:,1), -accLH(:,3), accLH(:,2)];
accC = [accC(:,1), -accC(:,3), accC(:,2)];

gyroRF = [gyroRF(:,1), -gyroRF(:,3), gyroRF(:,2)];
gyroRH = [gyroRH(:,1), -gyroRH(:,3), gyroRH(:,2)];
gyroLF = [gyroLF(:,1), -gyroLF(:,3), gyroLF(:,2)];
gyroLH = [gyroLH(:,1), -gyroLH(:,3), gyroLH(:,2)];
gyroC = [gyroC(:,1), -gyroC(:,3), gyroC(:,2)];

%% Attitude qC, ... are all q_b2G 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% --> to euler: [thx,thy,thz] = -quat2eul(q_b2g,'XYZ') 
%%% or [thz,thy,thx] = quat2eul(q_g2b,'ZYX')in which q_g2b = quatconj(q_b2g)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Madgwick observer
addpath('quaternion_library');      % include quaternion library
% [quaternion, yaw_nullified_quat, yaw_rate]
[qC, qnyC, yrC, UC] = getAttitude(gyroC, accC, Fs);
%%
[qRF, qnyRF, yrRF, URF] = getAttitude(gyroRF, accRF, Fs);
[qRH, qnyRH, yrRH, URH] = getAttitude(gyroRH, accRH, Fs);
[qLF, qnyLF, yrLF, ULF] = getAttitude(gyroLF, accLF, Fs);
[qLH, qnyLH, yrLH, ULH] = getAttitude(gyroLH, accLH, Fs);

%% Transform U's from G to Chest Frame
R_G2B_C = quat2dcm(quatconj(qC));
URS = permute(pagemtimes(R_G2B_C,'none',URF,'none'),[3,1,2]);
ULS = permute(pagemtimes(R_G2B_C,'none',ULF,'none'),[3,1,2]);
USP = permute(pagemtimes(R_G2B_C,'none',UC,'none'),[3,1,2]);
URFA = permute(pagemtimes(R_G2B_C,'none',URH,'none'),[3,1,2]);
ULFA = permute(pagemtimes(R_G2B_C,'none',ULH,'none'),[3,1,2]);
X_tU = [-URS, -ULS, USP, -URFA, -ULFA];
%% Save
% Train set 
X_t = [qRF, qLF, qC, qRH, qLH];
% Train set for nullified yaw
X_tn = [qnyRF, qnyLF, qnyC, qnyRH, qnyLH, yrRF, yrLF, yrC, yrRH, yrLH];

save('MyQuatTest.mat','X_t','X_tn','X_tU','R_G2B_C','dataA');

%% Get attitude
function [q_B2G, q_nonY_B2G, YawRate, U_G] = getAttitude(gyro, acc, Fs)

AHRS = MadgwickAHRS('SamplePeriod', 1/Fs, 'Beta', 0.1);
q = zeros(length(acc), 4);
for i = 1:length(acc)
    AHRS.UpdateIMU(gyro(i,:), acc(i,:));	% gyroscope units must be radians
    q(i, :) = AHRS.Quaternion;
end
% The q in Madgwich is q_G2B!
%eulE = quatern2euler(quaternConj(q));

%% Remove Yaw from quaternion (G2B)
Yaw = quat2eul(q,'ZYX');
plot(Yaw)
legend('z','y','x')
Yaw = Yaw(:,1);
q_Y = [cos(Yaw/2) 0.*Yaw 0.*Yaw sin(Yaw/2)];
q_nonY = quatmultiply(quatconj(q_Y), q);
%YawRate = [0;diff(Yaw)*Fs];
YawRate = gyro(:,3);
% eulE_nonY = quat2eul(q_nonY,'ZYX');
% figure(3)
% plot(eulE)
% legend('z','y','x')
%% Convert back to B2G
q_B2G = quatconj(q);
q_nonY_B2G = quatconj(q_nonY);
%% Convert to Rotm
R_B2G = quat2dcm(q_B2G);
%% Get U vector
U_G = R_B2G(:,3,:);
end
end
