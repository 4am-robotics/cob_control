function [] = svd_splitted_tasks(m, n, dampingFactor)
    clc; 

    global EPS;
    EPS = 1.0e-12 
    
    if nargin < 3
        dampingFactor = 0.0
    endif 

%    v = rand(m, 1);
v = [0.00193851;
6.23383e-07;
7.88073e-07;
7.79324e-07;
0.000806476;
-0.00129337
]

    % J = [1 -1 0; 0 1 1];
    % J = rand(m, n);

J = [2.10135e-07      -0.7522  1.16821e-07      -0.2986            0            0            0; 
    -0.000546374 -0.000151469 -8.29758e-05 -0.000420397  5.42101e-20            0            0; 
               0 -0.000546374  6.16749e-11  -0.00026693            0            0            0;
               0  0.000201371 -0.000616057    0.0014079  -0.00089394   0.00168954 -0.000765242;
               0           -1 -1.24055e-07    -0.999999 -5.15285e-07    -0.999999 -2.97845e-07;
               1 -3.67321e-06            1 -2.92992e-06            1 -2.67814e-06            1
]


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

    
    k = 55.0; 


    [U, singularValuesInv, V] = invSingularValueDecomposition(J, k);
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
        i        
        J_i = J(i, :)
        v_i = v(i, :);
        
        J_schlange = J_i * P
        [U, singularValuesInv, V] = invSingularValueDecomposition(J_schlange, k);
        J_pinv_i = V * singularValuesInv * U'
        
        
        q_i = q_i + J_pinv_i * (v_i - J_i * q_i);
        P = P - J_pinv_i * J_schlange; 
    end
    

    error_abs = abs(q_full - q_i)
    
    [r, c] = size(error_abs);
    
    for lv = 1 : r
        error_rel = error_abs(lv, 1) / abs(q_full(lv, 1))
    end
end
