%% EE 547 Project : Balancing the MinSeg Robot (Inverted Pendulum system)
%Authors : Anusha Kamat, Jyothi Venkoba Rao

close all;
clear;
clc;

%% Parameters of Minseg bot 

%Parameters to run the steps
syms t;
tspan = 0:0.1:5;
x0 = [-50;40;-300;20];  
x1 = [0;0;0;0];
xini =[0;0;0;0];
xinihat = xini;

%desired pole location 
% 1. Close to open loop poles - less control effort
% 2. Not very negative poles - have more reaction time

%choice of open pole poles
%P1 = [-140.8360, -5.4696, -40, -65]; 
%P1 = [-6.5,-4.1,-2+1.23j,-2-1.23j];%works best
%P1 = [-140.65,-5.1,-10,-15];%works good
%P1 = [ -36+10.23j,-36-10.23j,-350,-60];
%P1 = [-10.5 , -16.5+0.1j,-16.5-0.1j,-6];
P1 = [-6,-650,-30,-20]; % works best with 6 times faster dynamics

%Global values and flags
bot1 = 1;%Flag to select Robot 1
battery_on = 1; %flag for bot running by battery or usb supply. 1 - battery 0 - usb
g=9.81;

%Step 2 
% constant values for both the bot
R=5.2628 ;%ohms
k_b =0.4953 ;%Vs/rad
k_t=0.3233 ;%Nm/A

%% Parameters from Bot 1
L1=0.115;
r_w1=0.0215;
m_p1 =0.3455;
m_w1=0.0357;
i_p1=0.00616 ;
i_cmw1=1.49*10^-4;

%% Parameters from Bot 2
m_w2=0.0358; %kg
i_cmw2 = 0.0000174047;
r_w2=0.018;

%value of parameters changes with or without battery
m_p_batt =0.3383;
m_p_wobatt = 0.1998;
i_p_batt= 2.5172996;
i_p_wobatt = 2.3983477;
L_batt=0.122;
L_wobatt = 0.115;

%% Selection of Bot and parameters depending on bot1 and battery_on flag

if(bot1 ==1)
    m_w =m_w1;
    i_cmw = i_cmw1;
    r_w =r_w1;
    m_p = m_p1;
    i_p = i_p1;
    L = L1;
else
    m_w =m_w2;
    i_cmw = i_cmw2;
    r_w =r_w2;    
if(battery_on ==1)
    m_p = m_p_batt;
    i_p = i_p_batt;
    L = L_batt;
else
    m_p = m_p_wobatt;
    i_p = i_p_wobatt;
    L = L_wobatt;
end
end

%% Formation of matrices

den_1 = (i_cmw *(i_p +(L^2*m_p ))+(((L^2)* m_p* m_w)+i_p*(m_p+m_w))*r_w^2 ); %denominator for the A and B matrices

a11=0;
a12=1;
a13=0;
a14=0;

a21= ((g*L*m_p)*(i_cmw+(m_p+m_w)*(r_w^2 )))/den_1;
a22= -((k_b*k_t)* (i_cmw+r_w*(m_w*r_w+m_p *(L+r_w))))/(R*den_1);
a23= 0;
a24= a22/r_w;

a31=0;
a32=0;
a33=0;
a34=1;

a41=(g*L^2*m_p^2*r_w^2)/den_1;
a42=  -((k_b*k_t*r_w)* (i_p+ L*m_p*(L+r_w)))/(R*den_1);
a43=0;
a44=a42/r_w;

A=[a11 a12 a13 a14;
    a21 a22 a23 a24;
    a31 a32 a33 a34;
    a41 a42 a43 a44];

B=[0;
    -(k_t*(i_cmw+r_w*(m_w*r_w+m_p *(L+r_w))))/(R*den_1);
    0;
    -((k_t*r_w)* (i_p+ L*m_p*(L+r_w)))/(R*den_1)];
    

C = eye(size(A));
D = zeros(size(B));

%%  Step 1 :State space equations 
states = {'alpha' 'alpha_dot' 'x' 'x_dot' };
inputs = {'V'};
outputs = {'alpha' 'alpha_dot' 'x' 'x_dot'};

sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

%% Step 3 : Transfer functions of Linearized system

sys_tf = tf(sys_ss);
[num,den] = ss2tf(A,B,C,D);
tf1 = tf(num(1,:),den);
tf2 = tf(num(2,:),den);
tf3 = tf(num(3,:),den);
tf4 = tf(num(4,:),den);

%% Plots of the system
figure(1)
iopzplot(sys_ss)
grid on;
fn1_sys_responses(sys_tf);

%% Step 4 :  characteristic polynomial and eigenvalues of matrix A

ChaPoly = poly(A);   
rts = roots(ChaPoly); 
eigenvalues = eig(A);

%%  Step 5 : Asymptotic stability

fn2_asymptotic_stability(eigenvalues); %not asymptotically stable

%% Step 6 : poles and Zeros of system and BIBO stability

poles_tf1 = pole(tf1) ; %den is same for all tf
zeros_tf1 = zero(tf1) ;
zeros_tf2 = zero(tf2); 
zeros_tf3 = zero(tf3) ;
zeros_tf4 = zero(tf4) ;

fn3_bibo_stability(poles_tf1); %not bibo stable 

%% 4.2 : step 7,8: Controllability, Observability 
Ctrb_matrix = fn4_chk_control(A,B);
Obsv_matrix= fn5_chk_observe(A,C);

%% 4,2 :Step 9 : canonical form

sys_canon = canon(sys_ss, 'companion');


%To get diffrent forms of Cannonical form (OCF and CCF)
alpha = den(2:end); % denominator coefficients of sys_tf
%Formation of inverse of Cbar 
Ctrb_bar_inv = [1, alpha(1), alpha(2), alpha(3); 
              0, 1, alpha(1), alpha(2); 
              0, 0, 1, alpha(1);
              0, 0, 0, 1];
Q = Ctrb_matrix*Ctrb_bar_inv; 
A_bar = round(Q\A*Q*1e5)/1e5;
B_bar = round(Q\B*1e5)/1e5;
C_bar = round(C*Q*1e5)/1e5;
D_bar = D;
CCF = ss(A_bar, B_bar, C_bar, D_bar);
OCF = canon(sys_ss, 'companion');


%% PART 4.3 : State Estimator
%Step 10 : Full order observer for open loop system

L= fn6_state_estimator(sys_ss,P1,xini,xinihat);

%% PART 4.4 : Feedback Control by placing poles

%step 12 and 13: state space of closed loop
[ssCL,K] =fn7_feedback(sys_ss,P1,xini);
disp('The gain for stable closed loop');
disp(K);

ChaPolyCL = poly(ssCL.A);
eigCL = eig(ssCL.A);

%the placement of closed loop poles
figure(6); 
iopzplot(ssCL);
grid on;

%stability of feedback controller
disp('closed loop system');
fn2_asymptotic_stability(eigCL);

%step14 : Simulink for closed loop
sim('slx2_step14_fb', tspan(end));
figure(7);
plot(t, y,'Linewidth',3);
grid on;
ylabel('Output');
xlabel('time (sec)');
title('Step Response of Closed Loop system with prop fb controller');
legend('\alpha', '\alpha_d_o_t', 'x','x_d_o_t');


%% 4.5 Feedback control using state estimator
%Step 15 :Design of feedback controller is merged with state estimator
%giving us u = r-k*xhat for the plant 

fn8_obsv_feedback(sys_ss,L,C,xini);



%% LQR
Q = diag([100, 100, 100, 100]);
R =1;
Ts = 0.005;
[K,S,E] = lqr(A,B,Q,R);

KLQR = [K(3) K(4) K(1) K(2)];

ssLQR = fn7_feedback(sys_ss,K);
ssLQRd = c2d(ssLQR, Ts, 'zoh');
pole(ssLQRd);

fprintf('The gain from LQR from MinSeg stability is\n')
disp(KLQR);

