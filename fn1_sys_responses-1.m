function fn1_sys_responses(sys_tf)
t1=0:0.01:1;
figure(2);
impulse(sys_tf,t1);
title('Open-Loop Impulse Response');

t2 = 0:0.05:10;
u = ones(size(t2));
[y,t2] = lsim(sys_tf,u,t2);
figure(3);
plot(t2,y,'Linewidth',2);
title('Open-Loop Step Response');
legend('x','x_d_o_t','\alpha','\alpha_d_o_t','Location','southwest');