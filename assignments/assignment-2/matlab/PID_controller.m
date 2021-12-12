function PID = PID_controller(G)
    Am = 3;
    [rows, cols] = size(G);
    K = zeros(rows, cols);
    L = zeros(rows, cols);
    T = zeros(rows, cols);
    PID = cell(rows, cols);
    for i = 1 : rows
        K(i,i) = G(i,i).num{1}(end);
        L(i,i) = G(i,i).InputDelay + G(i,i).ioDelay;
        T(i,i) = G(i,i).den{1}(1);
        PID{i,i} = {pi * T(i, i) / 2 / Am / L(i, i) / K(i, i), pi / 2 / Am / L(i, i) / K(i ,i), 0};
    end
end