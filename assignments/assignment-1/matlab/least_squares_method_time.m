% Define the original transfer function 
s = tf('s');
G0 = 3 * exp(-2 * s) / ((s + 4) * (s^3 + 5*s^2 + 7*s + 4));

% Sampling
start_time = 2;
end_time = 32;
Ts = 0.01;
s_num = (end_time - start_time) / Ts;
t = start_time : Ts : end_time - Ts;
y = step(G0, t);

% Calculate psi and gamma
A = 1;
psi = zeros(s_num, 3);
psi(1, 1) = Ts / 2 * y(1);
for i =2 : s_num - 1
      psi(i, 1) = psi(i - 1, 1) + Ts*y(i);
end
psi(s_num, 1) = psi(s_num - 1, 1) + Ts / 2 * y(s_num);
psi(:, 1) = -psi(:, 1);
psi(:, 2) = -A;
psi(:, 3) = A * t;
gamma = y;

% Using least squares method to calculate the parameters
theta = ((psi' * psi)^-1) * psi' * gamma;
a1 = theta(1, 1);
b1 = theta(3, 1);
L = theta(2, 1) / theta(3, 1);

Gp = b1 * exp(- L * s) / (s + a1);
step(G0, Gp);
legend('Original','Frist order plus time delay model');
title('Least Squares Method - Time');
subtitle({'Frist order plus time delay model parameters', ['A = ', num2str(A), ', K =' num2str(b1 / a1), ', T =', num2str(1 / a1), ', L =', num2str(L)]});

figure(2);
nyquist(G0, Gp, {0.0001, 0.9});
legend('Original','Frist order plus time delay model');