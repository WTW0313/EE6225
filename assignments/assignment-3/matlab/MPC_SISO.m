function [U, J] = MPC_SISO(wk, xk, Phi, G, N1, N2, Nu, Lambda)
    cvx_begin quiet
        variable U(Nu)
        Y = Phi * xk + G * U;
        W = wk * ones(size(Y, 1), 1);
        OBJ = (W - Y)' * (W - Y) + Lambda * (U' * U);
        minimize(OBJ);
        
        % Add constrains on U
        UMIN = -2 * ones(size(U, 1), 1);
        UMAX = 2 * ones(size(U, 1), 1);
        subject to
            UMIN <= U <= UMAX;
    cvx_end
    J = cvx_optval;
end

