% NXT parameters:
Rm = 5.262773292;     % ohms
Kb = 0.4952900056;    % Vs/rad
Kt = 0.3233728703;    % Nm/A
Bm = 0.0006001689451; % Nms/rad (viscous friction)
Lm = 0.0047;          % H    
Jm = 0.001321184025;  % kgm^2 (combined J)
Tf = 0.007299397206;  % Nm (coulomb friction)
encoder_counts=720;   % number of counts (if using quad encoding)
RPM_MAX = 170;        % spec sheet max RPM
mp = .080170;         % Mass of Pendulum (NXT Motor assembly)
lp=0.07;              % pendulum length m

% assume inertia of disk with radius 2cm:
Ip=1/2*mp*.02^2;     % inertia of pendulum about center of mass

% constants or conversion factors:
RADSEC2RPM = 60/(2*pi);       % radians/sec to RPM
RAD2C=encoder_counts/(2*pi);  % conversion from radians to counts
C2RAD=1/RAD2C; 
C2DEG=360/encoder_counts;
R2D=180/pi;
D2R=1/R2D;
g=9.8;

% Wheel Info
Rw=.0216;  % radius of wheel in m (small wheel)
%Rw=.028;   % big 56mm wheels
%Rw=.015;   % small 30 mm wheels

% hardware
Vsupply=6;  % 9 volts
DCB2V=Vsupply/255;   % Duty cycle bits to PWM (volts)
V2DCB=1/DCB2V;       % Volts to duty cycle in bits.
GyroS=1/112;  % gyro scaling factor

% LQR Gain:
%  KLQR= [3.1623   46.1196  -50.6384   -5.8005]  % Q=10first working.  (okay on power supply)
% KLQR= [7.0711   49.2777  -66.2566   -9.5280] % Q=50  2.5ms still jittery on power supply.
% KLQR= [10.0000   51.6376  -78.2912  -12.5550] % 43mm*** Q=100 - even slower/smoother (on battery), on power supply very very jittery 2.5ms
 KLQR=[10.0000   42.1458  -68.8861  -12.0203]; % 56mm wheel .032kg Q=100
% KLQR=[ 10.0000   70.7001  -95.8299  -13.6744]; % 30mm small  wheel Q=100
% KLQR= [14.1421   55.6556  -96.1187  -16.9900] %Q=200  % jittery on
%battery but okay for when battery gets low (will improve as battery dies)
%KLQR= [20.0000   62.3023 -122.6693  -23.4192] %Q=400

TS=.0024  % fastest pololu gyro

% TS=.03
%TS=.0026; fastest with 8 bit serial with gyro
%TS=.0025; fastest with no serial with gyro
%TS=.0021; %fastest with 16 serial (no gyro)
%TS=.0011; %fastest with 8 bit serial (no gyro)
%TS=.0003; fastest with no serial (no gyro)

% combine some constants:
GS=GyroS*D2R;
ES=-C2DEG*D2R;
KLQRC=KLQR.*[-Rw -Rw 1 1]; % combined LQR Gain with wheel radius scaling
Ki=1*[-Rw -Rw*0 1 1*0];

% automatic calibration of gyro offset
% discrete 1st order filter recurrence relation - 
% discrete-time implementation of a low-pass filter is 
% the exponentially-weighted moving average
% alpha=.5 (time contant is equal to sampling period)
% alpha<.5 (time constant is larger than sampling period) tau ~ TS/alpha

% .006 requires 800 samples to get to steardy state (see gyro filter design)
% 1st order system take 3Tau to get to .95 ss value, 3.9Tau, 98, 4.56tau 99
% tau ~ .0025/.006 = .4167 = 3*.4167 = 1.25 to get to .95  - we need .99
% tau ~ .0025/.006 = .4167 = .4167*.456 = 1.9sec to get to 99% ss
a_go = .006;  % alpha for initial gyro offset calibration (2 sec for ss)

% Start time for balancing - Gyro calibartion time:
tstart=2;

% TS=.005;
% tstart=4;

% add the path to the common files for
% obtaining serial data and plotting:
addpath('C:\adunin\CommonFiles')


%% MPU5060
TS=.005 % fastest with default mpu5060 library settings... and filter set to DLPF set to 4
GyroS=1/131;   % MPU5060
GS=GyroS*D2R;
tstart=.6;
return
% Low pass filter for output:
a_ov = .7;  % on USB
a_ov= .5; % on battery
a_ov=1;  % no filter.

