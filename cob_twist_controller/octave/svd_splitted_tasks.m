function [] = svd_splitted_tasks(m, n, dampingFactor)
    clc; 

    global EPS;
    EPS = 1.0e-12 
    
    if nargin < 3
        dampingFactor = 0.0
    endif 

    v = rand(m, 1);

    % J = [1 -1 0; 0 1 1];
    J = rand(m, n);
    J_T = J'; 

    rankJ = rank(J)

    [rows, cols] = size(J)

    disp('Direct calculation');

    if rows < cols && rankJ == rows
        disp('=> Rows > Cols');
        J_pinv = J_T * inv(J * J_T);

    elseif rows > cols && rankJ == cols
        disp('=> Rows < Cols');
        J_pinv = inv(J_T * J) * J_T;

    elseif rows == cols && rankJ == rows
        disp('=> Rows == Cols');
        J_pinv = inv(J);

    else
        J_pinv = zeros(rows, cols);
        disp('ERROR due to singularity');

    end
    disp('Result: ');
    J_pinv

    

    [U, singularValuesInv, V] = invSingularValueDecomposition(J);
    J_svd_pinv = V * singularValuesInv * U'; 
    disp('Result: ');
    J_svd_pinv
    
    disp('Unsplitted result: ');
    q_full = J_svd_pinv * v 

    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Now split
    
    disp('Splitted result: ');
    q_i = zeros(n, 1); 
    P = eye(n, n); 
    for i = 1 : m
        J_i = J(i, :);
        v_i = v(i, :);
        
        J_schlange = J_i * P; 
        [U, singularValuesInv, V] = invSingularValueDecomposition(J_schlange);
        J_pinv_i = V * singularValuesInv * U'; 
        
        i
        q_i = q_i + J_pinv_i * (v_i - J_i * q_i)
        P = P - J_pinv_i * J_schlange; 
    end
    

    error_abs = abs(q_full - q_i)
    
    [r, c] = size(error_abs);
    
    for lv = 1 : r
        error_rel = error_abs(lv, 1) / abs(q_full(lv, 1))
    end
end
