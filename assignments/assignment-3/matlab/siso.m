clc;
clear;
bz = 0.1;
az = [1 -0.9];
ts = 0.1;
Gz = tf(bz, az, ts);
Gz = ss(Gz);
Ap = Gz.A;
Bp = Gz.B;
Cp = Gz.C;
Dp = Gz.D;
nx = size(Ap, 1);

model = ss(Ap, Bp, Cp, Dp, ts);
A = model.A;
B = model.B;
C = model.C;

N1 = 1;
N2 = 2;
Nu = 1;
Lambda = 0.01;

Phi = [C * A; C * A^2; C * A^3];
G = [C * B; C * A * B; C * A^2 * B];
tsim = 20;
SetPt = [0 ones(1, tsim / 2), zeros(1, tsim / 2)];

y_plot = zeros(1, tsim);
u_plot = zeros(1, tsim);
w_plot = zeros(1, tsim);
J_plot = zeros(1, tsim);

for k = 1 : tsim
    if mode(k, 10) == 0
        disp(k);
    end
    wk = SetPt(k);
    if k > 1
        xk = x;
    else
        xk = zeros(nx, 1);
    end
    yk = C * xk;
    [U, J] = MPC_SISO(wk, xk, Phi, G, N1, N2, Nu, Lambda);
    uk = U(1, :);
    x = Ap * xk + Bp * uk;
    y_plot(k) = yk;
    u_plot(k) = uk;
    w_plot(k) = wk;
    J_plot(k) = J;
end

subplot(3, 1, 1);
stairs(w_plot);
hold on;
plot(y_plot);
ylabel('Y');
subplot(3, 1, 2);
stairs(u_plot);
ylabel('U');

subplot(3, 1, 3);
stairs(J_plot);
ylabel('J');


