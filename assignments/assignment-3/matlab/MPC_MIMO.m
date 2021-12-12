function [U, J] = MPC_MIMO(wk, xk, Phi, G, m, N1, N2, Nu, Lambda, N_input, MIN, MAX)
    cvx_begin quiet
        variable U(m * Nu, 1)
        Y = Phi * xk + G * U;
        W = ones(size(Y, 1), 1) * wk(N_input);
        OBJ = (W - Y)' * (W - Y) + Lambda * (U' * U);
        minimize(OBJ);
        
        % Add constrains on U
        UMIN = MIN * ones(size(U, 1), 1);
        UMAX = MAX * ones(size(U, 1), 1);
        subject to
            UMIN <= U <= UMAX;
    cvx_end
    J = cvx_optval;
end

