% Plot the unit step function
syms x;
figure(1);
fplot(heaviside(x));
xlim([-1, 1]);
ylim([-0.2, 1.2]);
title('Unit Step Function')

% Plot the step response of original transfer function
s = tf('s');
G0 = 3 * exp(-2 * s) / ((s + 4) * (s^3 + 5*s^2 + 7*s + 4));
figure(2);
step(G0);

% Find t1 & t2
steadyStateY = 0.1875;
y1 = steadyStateY * 0.284;
y2 = steadyStateY * 0.632;
t = 0:0.001:20;
Y0 = step(G0, t);
figure(3);
plot(t, Y0);
title('step response');
xlabel('Time (seconds)');
ylabel('Amplitude');

% Define the transfer function of FOPTD model
t1 = 3.406;
t2 = 4.269;
A = 1;
K = steadyStateY / A;
T = 1.5 * (t2 - t1);
L = 0.5 * (3 * t1  - t2);
Gp = K * exp(- L * s) / (T * s + 1);

% Compare two transfer functions
figure(4);
step(G0, Gp, 20);
legend('Original','Frist order plus time delay model');
title('Two Points Method');
subtitle({'Frist order plus time delay model parameters', ['A = ', num2str(A), ', K =' num2str(K), ', T =', num2str(T), ', L =', num2str(L)]});

figure(5);
nyquist(G0, Gp, {0.0001, 0.9});
legend('Original','Frist order plus time delay model');