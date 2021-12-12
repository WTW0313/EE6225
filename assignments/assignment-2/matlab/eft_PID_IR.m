% ETFs and PID parameters without Integrity Rules
function [G_hat, PID] = eft_PID_IR(G, Lambda, Gamma)
    Am = 3;
    [rows, cols] = size(G);
    G_hat = [tf(0, 1), tf(0, 1), tf(0, 1);
        tf(0, 1), tf(0, 1), tf(0, 1);
        tf(0, 1), tf(0, 1), tf(0, 1)];
    K = zeros(rows, cols);
    L = zeros(rows, cols);
    T = zeros(rows, cols);
    PID = cell(rows, cols);
    for i = 1 : rows
        for j = 1 : cols
            K(i,j) = G(i,j).num{1}(end);
            L(i,j) = G(i,j).InputDelay + G(i,j).ioDelay;
            T(i, j) = G(i, j).den{1}(1);
            if abs(Lambda(i, j)) < 1
                if Gamma(i, j) > 1
                    G_hat(i, j) = tf(K(i, j) / Lambda(i, j), [Gamma(i, j) * T(i, j), 1], 'inputDelay', Gamma(i, j) * L(i, j));
                    PID{i, j} = [pi * Lambda(i, j) * T(i, j) / 2 / Am / L(i, j) / K(i, j), pi * Lambda(i, j) / 2 / Am / Gamma(i, j) / L(i, j) / K(i, j), 0];
                elseif Gamma(i, j) > 0 && Gamma(i, j) <= 1
                    G_hat(i, j) = tf(K(i, j) / Lambda(i, j), [T(i, j), 1], 'inputDelay', L(i, j));
                    PID{i, j} = [pi * Lambda(i, j) * T(i, j) / 2 / Am / L(i, j) / K(i, j), pi * Lambda(i, j) / 2 / Am / L(i, j) / K(i, j), 0];
                end
            else
                if Gamma(i, j) > 1
                    G_hat(i, j) = tf(sign(Lambda(i, j)) * K(i, j), [Gamma(i, j) * T(i, j), 1], 'inputDelay', Gamma(i, j) * L(i, j));
                    PID{i, j} = [pi * T(i, j) / 2 / Am / L(i, j) / K(i, j), pi / 2 / Am / Gamma(i, j) / L(i, j) / K(i, j), 0];
                elseif Gamma(i, j) > 0 && Gamma(i, j) <= 1
                    G_hat(i, j) = tf(sign(Lambda(i, j)) * K(i, j), [T(i, j), 1], 'inputDelay', L(i, j));
                    PID{i, j} = [pi * T(i, j) / 2 / Am / L(i, j) / K(i, j), pi / 2 / Am / L(i, j) / K(i, j), 0];
                end
            end
        end
    end
end