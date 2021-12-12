% Define the original transfer function 
s = tf('s');
G0 = 3 * exp(-2 * s) / ((s + 4) * (s^3 + 5*s^2 + 7*s + 4));

% Sampling on the original step response
start_time = 0;
end_time = 20;
Ts = 0.1;
s_num = (end_time - start_time) / Ts;
t_sample = start_time : Ts : end_time - Ts;
[y, t] = step(G0, t_sample);

% Define the trapezoidal integration
syms w;
g(w) = (y(s_num) + w * trapz(t, (step(G0, t) - y(s_num)) .* sin(w * t))) + 1i * w * trapz(t, (step(G0, t) - y(s_num)) .* cos(w * t));

% Set initial values
M = 10;
W = zeros(1, M);
fai = zeros(1, M);
psi = zeros(M, 2);
gamma = zeros(M, 1);

W(1) = 0;
W(2) = 10^-3;
fai(1) = 0;
fai(2) = angle(g(W(2)));

% Recursive solution
for i = 3 : M
    W(i) = W(i - 1) - ((i - 1) * pi / (M - 1) + fai(i - 1)) * (W(i - 1) - W(i - 2)) / (fai(i - 1) - fai(i - 2));
    fai(i) = angle(g(W(i)));
end

% Calculate a1 and b1 by least squares method
for n = 1 : M
    psi(n, 1) = -(abs(g(W(n)))^2);
    psi(:, 2) = 1;
    gamma(n) = (W(n)^2) * (abs(g(W(n)))^2);
end
theta = ((psi' * psi)^-1) * psi' * gamma;
a1 = sqrt(theta(1));
b1 = sqrt(theta(2));

% Calculate L by least squares method
psi_L = W';
gamma_L = zeros(M, 1);
for n = 1 : M
    gamma_L(n) = -fai(n) - atan(W(n) / a1);
end
L = ((psi_L' * psi_L)^-1) * psi_L' * gamma_L;

A = 1;
Gp = b1 * exp(- L * s) / (s + a1);
step(G0, Gp);
legend('Original','Frist order plus time delay model');
title('Least Squares Method - Frequency');
subtitle({'Frist order plus time delay model parameters', ['A = ', num2str(A), ', K =' num2str(b1 / a1), ', T =', num2str(1 / a1), ', L =', num2str(L)]});

figure(2);
nyquist(G0, Gp, {0.0001, 0.9});
legend('Original','Frist order plus time delay model');

