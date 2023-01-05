function [angleE] = Attitude_Estimation3D(a_body, g_body, yaw_ref, K1, K2, K3, TH, Fs)
%% Kalman Filter Angle Estimation
T = 1/Fs;
g = sqrt( a_body(1,1)^2 +  a_body(1,2)^2 + a_body(1,3)^2 );

%initial mounting angle in degs
pitch0 = sign(a_body(1,1))*acos( a_body(1,2)/g );
%roll0 = sign(-a_body(1,3))*acos( a_body(1,3)/g );
roll0 = atan2(a_body(1,3), a_body(1,2));
% another way
% tilt0 = atan2(a_body(1,1), sqrt(a_body(1,2)^2 + a_body(1,3)^2 ));
% roll0 = atan2(-a_body(1,3), a_body(1,2));

%time
t = 0:T:T*(length(a_body(:,1))-1);

%% Filter data
%low pass filter 4Hz and correct scale and direction
a_body  = lowpass(a_body, 4, 100);

% remove bias from gyro data
%g_body  = g_body - lowpass(g_body, 0.1, 100);

pitchRate = g_body(:,3);
rollRate = g_body(:,1);
yawRate = g_body(:,2);
%% Without KF
if sum(yaw_ref) == 0
    K3 = 0;
end
%% Kalman Filter
pitchE = zeros(1,length(t));
rollE = zeros(1,length(t));
yawE = zeros(1,length(t));
pitchE(1) = pitch0;
rollE(1) = roll0;
for i=2:1:length(t)
    ge = sqrt( a_body(i,1)^2 +  a_body(i,2)^2 + a_body(i,3)^2);
    roll_acc = atan2(a_body(i,3), a_body(i,2));%sign(a_body(i,2))*atan(-a_body(i,3)/a_body(i,2));
    tilt_acc = sign(a_body(i,1))*acos( a_body(i,2)/ge )*cos(roll_acc);
    roll_acc = sign(-a_body(i,3))*asin( a_body(i,3)/ge )*cos(tilt_acc);
    tilt_acc = sign(a_body(i,1))*acos( a_body(i,2)/ge )*cos(roll_acc);
%     roll_acc = sign(-a_body(i,3))*asin( a_body(i,3)/ge )*cos(tilt_acc);
%     tilt_acc = sign(a_body(i,1))*acos( a_body(i,2)/ge )*cos(roll_acc);
    K11 = K1;
    K21 = K2;
    K31 = K3;
    TH1 = TH;
    % Correction on gains for high acceleration movements
    if ge>15
%         tilt_acc = pitchE(i-1);
%         roll_acc = rollE(i-1);
          K11 = K1 * 2;   % See more tilts for biking for example
          %K21 = K2 * 4;  % Unnecessary?
          K31 = K3 * 50;  % Penalize yaw of legs strongly with chest
          TH1 = TH * 20 ; % Avoid Yaw drift
    end
    if abs(yawRate(i))<TH1
        yawRate(i) = 0;
    end
    pitchE(i) = pitchE(i-1) + (pitchRate(i) + K11 * (tilt_acc - pitchE(i-1))) * T;
    rollE(i) = rollE(i-1) + (rollRate(i) + K21 * (roll_acc - rollE(i-1))) * T;
    yawE(i) = yawE(i-1) + (yawRate(i) + K31 * (yaw_ref(i) - yawE(i-1))) * T;
end
angleE = [rollE',yawE',pitchE'];
