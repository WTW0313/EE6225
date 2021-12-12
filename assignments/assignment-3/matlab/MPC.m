function [y1, y2, y3, u1, u2, u3, j1, j2, j3] = MPC(N1, N2, Nu, Lambda)
    % Define the transfer function matrix 
    Gs = [tf(-0.98, [12.5, 1], 'inputDelay', 17), tf(-0.36, [15, 1], 'inputDelay', 27), tf(-0.14, [15.2, 1], 'inputDelay', 32);
        tf(-0.43, [14.7, 1], 'inputDelay', 25), tf(-0.92, [13, 1], 'inputDelay', 16), tf(-0.11, [15.6, 1], 'inputDelay', 33);
        tf(-0.12, [15, 1], 'inputDelay', 31), tf(-0.16, [15, 1], 'inputDelay', 34), tf(-1.02, [11.8, 1], 'inputDelay', 16)].';

    % Descretize Gs with sampling time ts
    ts = 2.5;
    Gz = c2d(Gs, ts);

    % Get state space model for MPC design
    Gz = absorbDelay(Gz);
    Gz = ss(Gz);
    Ap = Gz.A;
    Bp = Gz.B;
    Cp = Gz.C;
    Dp = Gz.D;
    n = size(Ap, 1);
    m = size(Bp, 2);
    p = size(Cp, 1);

    % Design MPC
    model = ss(Ap, Bp, Cp, Dp, ts);

    Phi = zeros((N2 - N1 + 1) * p, n);
    G = zeros((N2 - N1 + 1) * p, m * Nu);

    for i = N1 : N2
        Phi((i - 1) * p + 1 : i * p, :) = Cp * Ap^i;
        for k = 1 : Nu
            if i - k >= 0
                G((i - 1) * p + 1 : i * p, m * (k - 1) + 1 : m * k) = Cp * Ap^(i - k) * Bp;
            else
                break;
            end
        end
    end
    
    % Prediction horizon
    tsim = 200;
    
    SetPt = ones(tsim, 3);
    SetPt(1 : 50, 2) = 0;
    SetPt(1 : 100, 3) = 0;

    x = zeros(n, 1);

    y1 = zeros(tsim, 1);
    y2 = zeros(tsim, 1);
    y3 = zeros(tsim, 1);
    u1 = zeros(tsim, 1);
    u2 = zeros(tsim, 1);
    u3 = zeros(tsim, 1);
    j1 = zeros(tsim, 1);
    j2 = zeros(tsim, 1);
    j3 = zeros(tsim, 1);

    for k = 1 : tsim
        wk = SetPt(k, :);
        xk = x;
        yk = Cp * xk;

        % Compute the MPC control signal
        % Constrained
        [U1, J1] = MPC_MIMO(wk, xk, Phi, G, m, N1, N2, Nu, Lambda, 1, -4, 4);
        [U2, J2] = MPC_MIMO(wk, xk, Phi, G, m, N1, N2, Nu, Lambda, 2, -4, 4);
        [U3, J3] = MPC_MIMO(wk, xk, Phi, G, m, N1, N2, Nu, Lambda, 3, -4, 4);
        if k <= 50
            uk = [U1(1, 1), 0, 0];
        elseif k <= 100
            uk = [U1(1, 1), U2(1, 1), 0];
        else
            uk = [U1(1, 1), U2(1, 1), U3(1, 1)];
        end

        x = Ap * xk + Bp * uk';

        y1(k) = yk(1, 1);
        y2(k) = yk(2, 1);
        y3(k) = yk(3, 1);
        u1(k) = uk(1);
        u2(k) = uk(2);
        u3(k) = uk(3);
        j1(k) = J1;
        j2(k) = J2;
        j3(k) = J3;
    end
end