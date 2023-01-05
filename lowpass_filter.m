function [outputArg1,outputArg2] = lowpass_filter(inputArg1,inputArg2)
%LOWPASS_FILTER Summary of this function goes here
Ts = 0.001;                                                     % Sampling Interval (s)
Fs = 1/Ts;                                                      % Sampling Frequency (Hz)
Fn = Fs/2;                                                      % Nyquist Frequency (Hz)
Wp = 0.001;                                                     % Passband Frequency For Lowpass Filter (Hz)
Ws = 0.0012;                                                    % Stopband Frequency For Lowpass Filter (Hz)
Rp =  1;                                                        % Passband Ripple For Lowpass Filter (dB)
Rs = 50;                                                        % Stopband Ripple (Attenuation) For Lowpass Filter (dB)
[n,Wp] = ellipord(Wp,Ws,Rp,Rs);                                 % Calculate Filter Order
[z,p,k] = ellip(n,Rp,Rs,Wp);                                    % Calculate Filter
[sos,g] = zp2sos(z,p,k);                                        % Second-Order-Section For Stability
% A_Filt = filtfilt(sos,g,A);                                     % Filter Signal

end

