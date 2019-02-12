%% Feedback system 

function [ssCL,K] = fn7_feedback(sys,P1,xini)
A = sys.A;
B = sys.B;
C = sys.C;
D = sys.D;
tspan = 0:0.1:5;

K = place(A, B, P1); % the gain with new desired poles for this system
ACL = A - B*K;
ssCL = ss(ACL,B,C,D);

end