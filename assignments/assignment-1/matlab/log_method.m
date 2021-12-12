% Sampling on the original transfer function
s = tf('s');
G0 = 3 * exp(-2 * s) / ((s + 4) * (s^3 + 5*s^2 + 7*s + 4));
t = 2 : 0.1 : 6 - 0.1;
y = step(G0, t);

% Transformation of the step-response data against time t
k = size(y);
yln = [];
tln = [];
for n = 1 : 1 : k
    if y(n, 1) > 0 && 0.1875 - y(n, 1) >= 0
        yln = [yln, log((0.1875 - y(n, 1)) / 0.1875)];
        tln = [tln, t(1, n)];
    end
end

% Do linear fitting
h = polyfit(tln', yln', 1);
figure(1);
plot(tln, yln, '*', tln, polyval(h, tln));
grid on;
title('Linear Fitting');
subtitle(['y = ', num2str(h(1,1)), 'x + ', num2str(h(1,2))]);
xlabel('t', 'FontWeight', 'bold');
ylabel('ln((y_{∞} - y) / y_{∞})', 'FontWeight', 'bold');

% Define the transfer function of FOPTD model
steadyStateY = 0.1875;
A = 1;
K = steadyStateY / A;
T = 1 / -h(1, 1);
L = h(1, 2) * T;
figure(2);
Gp = K * exp(- L * s) / (T * s + 1);
step(G0, Gp);
legend('Original','Frist order plus time delay model');
title('Log Method');
subtitle({'Frist order plus time delay model parameters', ['A = ', num2str(A), ', K =' num2str(K), ', T =', num2str(T), ', L =', num2str(L)]});

figure(3);
nyquist(G0, Gp, {0.0001, 0.9});
legend('Original','Frist order plus time delay model');