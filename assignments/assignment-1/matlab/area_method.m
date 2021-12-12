% Sampling on the original step response
s = tf('s');
G0 = 3 * exp(-2 * s) / ((s + 4) * (s^3 + 5*s^2 + 7*s + 4));
figure(1);
step(G0);
Ts = 0.01;  % sampling interval
start_time_0 = 0;
end_time_0 = 12;
s_num_0 = (end_time_0 - start_time_0) / Ts; % total number of samples
t0 = start_time_0 : Ts : end_time_0 - Ts;
y0 = step(G0, t0);

% Calculate A0
steadyStateY = 0.1875;
A0 = Ts / 2 * (steadyStateY - y0(1));
for i = 2 : s_num_0 - 1
    A0 =  A0 + Ts * (steadyStateY - y0(i));
end
A0 = A0 + Ts / 2 * (steadyStateY - y0(s_num_0));

% Calculate A1
A = 1;
K = steadyStateY / A;
Tar = A0 / K;
start_time_1 = 0;
end_time_1 = floor(Tar * (1 / Ts)) / 100;
s_num_1 = (end_time_1 - start_time_1) / Ts;
t1 = start_time_1 : Ts : end_time_1 - Ts;
y1 = step(G0, t1);
A1 = Ts / 2 * y1(1);
for i = 2 : s_num_1 - 1
    A1 =  A1 + Ts * y1(i);
end
A1 = A1 + Ts / 2 * y1(s_num_1);

% Define the transfer function of FOPTD model 
T = (exp(1) * A1) / K;
L = (A0 / K) - ((exp(1) * A1) / K);
Gp = K * exp(- L * s) / (T * s + 1);
figure(2);
step(G0, Gp);
legend('Original','Frist order plus time delay model');
title('Area Method');
subtitle({'Frist order plus time delay model parameters', ['A = ', num2str(A), ', K =' num2str(K), ', T =', num2str(T), ', L =', num2str(L)]});

figure(3);
nyquist(G0, Gp, {0.0001, 0.9});
legend('Original','Frist order plus time delay model');