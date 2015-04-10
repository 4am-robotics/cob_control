function [] = svd_pseudoinverse(n, m, dampingFactor)
    clc; 

    global EPS;
    EPS = 1.0e-12 
    
    if nargin < 3
        dampingFactor = 0.0
    endif 

    % ---------- SR-Inverse with Weighting / JLA 2nd Part ----------------------------------
    disp('---------- SR-Inverse with Weighting / JLA ------------------------')    
%    J = rand(n, m)

    J = [1 2 3 4 5; 
         3 2 -5 2 1; 
         3 2 -5 2 1]
    J_T = J'; 

    % Generate Weighting
    [rowsJ, colsJ] = size(J);
    w_jla = ones(colsJ, 1); 
    for i = 1 : colsJ
        w_jla(i) = i * rand(1); 
    end    
    
    W_JLA = diag(w_jla);
        
    W_r = W_JLA^(-1/2)


    new_J = J * W_JLA^(-1/2); 
    new_J_T = new_J'; 
    
    matProd_J = new_J * new_J_T; 
    

    disp('1) Direct solution using matrix multiplication')    
    J_pinv = new_J_T * inv(matProd_J + dampingFactor * eye(size(matProd_J)))


    [U, D, V] = svd(J, 'econ'); % SVD of the default J is desired
    disp('2) Solution using SVD of default J and multiply with W^(-1/2)')    
%    J_pinv = W_r * V * D' * inv(D * V' * W_r * W_r * V * D' + dampingFactor * eye(size(D))) * U'
    V_r = W_r * V;
    J_pinv = V_r * D' * inv(D * V_r' * V_r * D' + dampingFactor * eye(size(D))) * U'

    [U, singularValuesInv, V] = invSingularValueDecomposition(new_J, dampingFactor);
    disp('3) Solution using the new_J = J * W^(-1/2) for SVD')    
    J_svd_pinv = V * singularValuesInv * U'
end

% tempSinv = 1 ./ S; does not work for all matrices 
% tempSinv(tempSinv == Inf) = 0.0; 
% singularValuesInv = tempSinv'; 


