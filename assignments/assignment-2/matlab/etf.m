% ETFs without Integrity Rules
function G_hat = etf(G, Lambda, Gamma )
    [rows, cols] = size(G);
    G_hat = [tf(0, 1), tf(0, 1), tf(0, 1);
        tf(0, 1), tf(0, 1), tf(0, 1);
        tf(0, 1), tf(0, 1), tf(0, 1)];
    K = zeros(rows, cols);
    L = zeros(rows, cols);
    T = zeros(rows, cols);
    for i = 1 : rows
        for j = 1 : cols
        K(i,j) = G(i,j).num{1}(end);
        L(i,j) = G(i,j).InputDelay + G(i,j).ioDelay;
        T(i, j) = G(i, j).den{1}(1);
        G_hat(i,j) = tf(K(i,j) / Lambda(i,j), [Gamma(i,j) * T(i,j), 1], 'inputDelay', Gamma(i,j) * L(i,j));
        end
    end
end