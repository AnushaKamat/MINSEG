function L = fn6_state_estimator(sys,P1,xini,xinihat)

A = sys.A;
B = sys.B;
C = sys.C;
D = sys.D;
tspan = 0:0.1:5;

%eigen values of (A'-C'K)' is used to determine observer gain where L = K'

L = place(transpose(A), transpose(C), P1); %for each pole, we find gain. % so for 3 poles 3 gains
L = transpose(L);

%Step 11: Plot of state variable and observers
sim('slx1_step11_obs', tspan(end));
%Plot of state variables, estimated states , error between actual state and estimated states and output response
figure(4); 
subplot(3,1,1);
plot(t, x, 'LineWidth', 1.5);
title('Closed loop state estimator for open loop minseg system');
xlabel('time (sec)');
ylabel('X');
legend('\alpha', '\alpha_d_o_t', 'x','x_d_o_t','Location','northwest');
%xlim([3 5]);

subplot(3,1,2);
plot(t, x_hat, 'LineWidth', 1.5);
xlabel('time (sec)');
ylabel('estimated X');
legend('\alpha_,_o_b_s', '\alpha_d_o_t_,_o_b_s', 'x_,_o_b_s','x_d_o_t_,_o_b_s','Location','northwest');
%xlim([3 5]);

subplot(3,1,3);
plot(t, y, 'LineWidth', 1.5);
xlabel('time (sec)');
ylabel('output');
legend('\alpha', '\alpha_d_o_t', 'x','x_d_o_t','Location','northwest');
%xlim([3 5]);


%xlim([3 5]);
for i=1:1:size(x,1)
    for j=1:1:size(x,2)
        error(i,j)=x(i,j)-x_hat(i,j);
    end
end

figure(5);
plot(t, error, 'LineWidth', 1.5);
xlabel('time (sec)');
ylabel('error');
title('The error between measured and estimated state variables with state estimator');
legend('\alpha_e', '\alpha_d_o_t_,_e', 'x_e','x_d_o_t_,_e','Location','northwest');
%xlim([3 5]);
end

