clc;
clear;
% Define the transfer function matrix obtained by empirical modeling technique
G = [tf(-0.98, [12.5, 1], 'inputDelay', 17), tf(-0.36, [15, 1], 'inputDelay', 27), tf(-0.14, [15.2, 1], 'inputDelay', 32);
    tf(-0.43, [14.7, 1], 'inputDelay', 25), tf(-0.92, [13, 1], 'inputDelay', 16), tf(-0.11, [15.6, 1], 'inputDelay', 33);
    tf(-0.12, [15, 1], 'inputDelay', 31), tf(-0.16, [15, 1], 'inputDelay', 34), tf(-1.02, [11.8, 1], 'inputDelay', 16)];

[rows, cols] = size(G);
K = zeros(rows, cols);
L = zeros(rows, cols);
T = zeros(rows, cols);
K_N = zeros(rows, cols);
for i = 1 : rows
    for j = 1 : cols
    K(i, j) = G(i, j).num{1}(end);
    L(i, j) = G(i, j).InputDelay + G(i, j).ioDelay;
    T(i, j) = G(i, j).den{1}(1);
    K_N(i,j) = K(i, j) / ( T(i, j) + L(i, j));
    end
end

% Calculate RGA
Lambda = K .* (K^-1)';
NI = det(K) / (K(1, 1) * K(2, 2) * K(3, 3));

% Calculate RNGA
Lambda_N = K_N .* (K_N^-1)';
Gamma = Lambda_N ./ Lambda;

% Without Integrity Rules
ETF = etf(G, Lambda, Gamma);

% With Integrity Rules
[ETF_IR, PID_IR] = eft_PID_IR(G, Lambda, Gamma);

% Simulate Decentralized Control
simout_decentralized = sim('Decentralized_PID_Controller');
% plot(simout_decentralized);

% Calculate interaction index matrix
Chi = NII(Lambda_N);

% Simulate Decentralized Control
simout_sparse = sim('Sparse_PID_Controller');
% plot(simout_sparse);

%Decoupling Control with Integrity Rules
G_R_IR = [tf([ETF_IR(1, 1).num{1}], [ETF_IR(1, 3).den{1}], 'inputDelay', ETF_IR(1, 3).InputDelay + ETF_IR(1, 3).ioDelay), 0, 0;
    0, tf([ETF_IR(2, 2).num{1}], [ETF_IR(2, 3).den{1}], 'inputDelay', ETF_IR(2, 3).InputDelay + ETF_IR(2, 3).ioDelay), 0;
    0, 0, tf([ETF_IR(3, 3).num{1}], [ETF_IR(3, 2).den{1}], 'inputDelay', ETF_IR(3, 2).InputDelay + ETF_IR(3, 2).ioDelay)];

G_hat_I_IR = [tf(0, 1), tf(0, 1), tf(0, 1);
        tf(0, 1), tf(0, 1), tf(0, 1);
        tf(0, 1), tf(0, 1), tf(0, 1)];

for i = 1 : rows
    for j = 1 : cols
        G_hat_I_IR(i, j) = tf(G_R_IR(j, j).num{1}(end) / ETF_IR(j, i).num{1}(end) * ETF_IR(j, i).den{1}, G_R_IR(j, j).den{1}, ...
        'inputDelay', abs(G_R_IR(j, j).InputDelay) + abs(G_R_IR(j, j).ioDelay) - abs(ETF_IR(j, i).InputDelay)- abs(ETF_IR(j, i).ioDelay));
    end
end

PID_decoupling_IR = PID_controller(G_R_IR);
simout_Decoupling_IR = sim('Decoupling_PID_Controller_IR');
%plot(simout_Decoupling_IR);

%Decoupling Control without Integrity Rules
G_R = [tf([ETF(1, 1).num{1}], [ETF(1, 1).den{1}], 'inputDelay', ETF(1, 3).InputDelay + ETF(1, 3).ioDelay), 0, 0;
    0, tf([ETF(2, 2).num{1}], [ETF(2, 2).den{1}], 'inputDelay', ETF(2, 2).InputDelay + ETF(2, 2).ioDelay), 0;
    0, 0, tf([ETF(3, 3).num{1}], [ETF(3, 3).den{1}], 'inputDelay', ETF(3, 3).InputDelay + ETF(3, 3).ioDelay)];

G_hat_I = [tf(0, 1), tf(0, 1), tf(0, 1);
        tf(0, 1), tf(0, 1), tf(0, 1);
        tf(0, 1), tf(0, 1), tf(0, 1)];

for i = 1 : rows
    for j = 1 : cols
        G_hat_I(i, j) = tf(G_R(j, j).num{1}(end) / ETF(j, i).num{1}(end) * ETF(j, i).den{1}, G_R(j, j).den{1}, ...
        'inputDelay', abs(G_R(j, j).InputDelay) + abs(G_R(j, j).ioDelay) - abs(ETF(j, i).InputDelay)- abs(ETF(j, i).ioDelay));
    end
end

PID_decoupling = PID_controller(G_R);
simout_Decoupling = sim('Decoupling_PID_Controller');
%plot(simout_Decoupling);

% Bode plot
%figure(1), margin(-G(1, 1));
%figure(2), margin(-G(2, 2));
%figure(3), margin(-G(3, 3));

% Calculate Ku and Wu
all_margin = [allmargin(-G(1, 1)), allmargin(-G(2, 2)), allmargin(-G(3, 3))];
Ku = [-all_margin(1).GainMargin(1), -all_margin(2).GainMargin(1), -all_margin(3).GainMargin(1)];
Wu = [all_margin(1).GMFrequency(1), all_margin(2).GMFrequency(1), all_margin(3).GMFrequency(1)];

K_ZN = Ku ./ 2.2;
T_ZN = 2 * pi ./ (1.2 * Wu);

min_error = 10^9;
N = 3;
for F = 2 : 0.05 : 5
    K_C = K_ZN / F;
    T_I = F * T_ZN;
    K_I = K_C ./ T_I;
    max_Lc = 0;
    Gc = [tf([K_C(1), K_I(1)], [1, 0]), 0, 0;
        0, tf([K_C(2), K_I(2)], [1, 0]), 0;
        0, 0, tf([K_C(3), K_I(3)], [1, 0])];
    for w = 0.1 : 0.01 : 1
        W = -1 + det(eye(3) + freqresp(G * Gc, w));
        Lc = 20 * log10(abs(W / (1 + W)));
        if Lc > max_Lc
             max_Lc = Lc;
        end
    end
    error = abs(max_Lc - 2 * N);
    if error < min_error
        min_error = error;
        F_match = F;
    end 
end

K_C = K_ZN / F_match; % P
T_I = F_match * T_ZN;
K_I = K_C ./ T_I; % I
simout_Decentralized_BLT = sim('Decentralized_PID_Controller_BLT');
%plot(simout_Decentralized_BLT);

for i = 1 : 3
    figure(3 + i), plot(simout_decentralized.Decentralized_ETF.Time, getcolumn(simout_decentralized.Decentralized_ETF.Data, i),'r');
    hold on;
    plot(simout_sparse.Sparse_ETF.Time, getcolumn(simout_sparse.Sparse_ETF.Data, i), 'y');
    hold on;
    plot(simout_Decoupling.Decoupling.Time, getcolumn(simout_Decoupling.Decoupling.Data, i), 'b');
    hold on;
    plot(simout_Decentralized_BLT.Decentralized_BLT.Time, getcolumn(simout_Decentralized_BLT.Decentralized_BLT.Data, i), 'g');
    title(['Comparison - output y', num2str(i)]);
    legend('decentralized', 'sparce', 'decoupling', 'BLT'); 
end
