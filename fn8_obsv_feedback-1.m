function fn8_obsv_feedback(sys,L,C,xini)

A = sys.A;
B = sys.B;
C = sys.C;
D = sys.D;
tspan = 0:0.1:5;
xinihat=xini;

%sim('xsh',tspan(end));
sim('slx3_step15_obsv_fb',tspan(end));
%error calc
for i=1:1:size(xf,1)
    for j=1:1:size(xf,2)
        error(i,j)=xf(i,j)-xhatf(i,j);
    end
end
figure(8);
plot(t,error,'Linewidth',2); %plotting error x-xhat
grid on;
xlabel('time (sec)');
ylabel('error');
ylim([-1*10^-14,1*10^-14]);
title('State Estimator with Feedback -error between measured and estimated state variables');
legend('x_1_e','x_2_e','x_3_e','x_4_e');

figure(9);
subplot(311);
plot(t, xf,'Linewidth',2);
title('State Estimator with Feedback -State variables');
grid on;
xlabel('time (sec)');
legend('x_1', 'x_2', 'x_3','x_4');

subplot(312);
plot(t, xhatf,'Linewidth',2);
title('State variables (estimated)');
grid on;
xlabel('time (sec)');
legend('x_1_,_o_b_s', 'x_2_,_o_b_s', 'x_3_,_o_b_s','x_4_,_o_b_s');

subplot(313);
plot(t, yf,'Linewidth',2);
title('Output State variables (measured)');
grid on;
xlabel('time (sec)');
legend('y_1','y_2','y_3','y_4');

end