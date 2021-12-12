% Define the original transfer function 
s = tf('s');
G0 = 3 * exp(-2 * s) / ((s + 4) * (s^3 + 5*s^2 + 7*s + 4));
K = 0.1875;
T = 1.0763;
L = 2.4415;
Gp = K * exp(- L * s) / (T * s + 1);

% Sampling on the original step response
start_time = 0;
end_time = 20;
Ts = 0.1;
s_num = (end_time - start_time) / Ts;
t_sample = start_time : Ts : end_time - Ts;
[y0, t0] = step(G0, t_sample);
[yp, tp] = step(Gp, t_sample);

% Define the trapezoidal integration
syms w;
g0(w) = (y0(s_num) + w * trapz(t0, (step(G0, t0) - y0(s_num)) .* sin(w * t0))) + 1i * w * trapz(t0, (step(G0, t0) - y0(s_num)) .* cos(w * t0));
gp(w) = (yp(s_num) + w * trapz(tp, (step(Gp, tp) - yp(s_num)) .* sin(w * tp))) + 1i * w * trapz(tp, (step(Gp, tp) - yp(s_num)) .* cos(w * tp));

% Set initial values
M = 10;
W0 = zeros(1, M);
Wp = zeros(1, M);
fai0 = zeros(1, M);
faip = zeros(1, M);

W0(1) = 0;
W0(2) = 10^-3;
Wp(1) = 0;
Wp(2) = 10^-3;
fai0(1) = 0;
fai0(2) = angle(g0(W0(2)));
faip(1) = 0;
faip(2) = angle(gp(Wp(2)));

% Recursive solution
for i = 3 : M
    W0(i) = W0(i - 1) - ((i - 1) * pi / (M - 1) + fai0(i - 1)) * (W0(i - 1) - W0(i - 2)) / (fai0(i - 1) - fai0(i - 2));
    fai0(i) = angle(g0(W0(i)));
    Wp(i) = Wp(i - 1) - ((i - 1) * pi / (M - 1) + faip(i - 1)) * (Wp(i - 1) - Wp(i - 2)) / (faip(i - 1) - faip(i - 2));
    faip(i) = angle(gp(Wp(i)));
end

real_part0 = zeros(1, M);
imag_part0 = zeros(1, M);
real_partp = zeros(1, M);
imag_partp = zeros(1, M);
for n = 1 : M
    real_part0(n) = real(g0(W0(n)));
    imag_part0(n) = imag(g0(W0(n)));
    real_partp(n) = real(gp(Wp(n)));
    imag_partp(n) = imag(gp(Wp(n)));
end
plot(real_part0, imag_part0, 'b-o');
grid on;
hold on;
plot(real_partp, imag_partp, 'r--*');
title('Nyquist chart');
xlabel('Real Axis');
ylabel('Imaginary Axis');
legend('Original', 'Relay feedback');
