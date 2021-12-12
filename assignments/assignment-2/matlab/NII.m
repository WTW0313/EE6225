function index_matrix = NII(Lambda_N)
    [rows, cols] = size(Lambda_N);
    index_matrix = zeros(rows, cols);
    diagonal = zeros(1, rows);
    for i = 1 : rows
        for j = 1 : cols
            if i == j
                diagonal(1, i) = Lambda_N(i, j);
                break;
            end
        end
    end
    for i = 1 : rows
        for j = 1 : cols
            index_matrix(i, j) = abs(Lambda_N(i, j) / diagonal(1, i));
        end
    end
end