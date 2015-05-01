function [] = WLN_JLA_PINV_SVD(n, m, dampingFactor)
    clc; 

    global EPS;
%    EPS = 1.0e-12
    EPS = 1.0e-1  
    
    if nargin < 3
        dampingFactor = 0.0
    endif 
    
    v_in = rand(3, 1);

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


    new_J = J * W_r; 
    new_J_T = new_J'; 
    
    matProd_J = new_J * new_J_T; 
    

    disp('1) Direct solution using matrix multiplication  (without TRUNCATION)')    
    J_pinv = new_J_T * inv(matProd_J + dampingFactor * eye(size(matProd_J))) % WITHOUT TRUNCATION!!!


    [U, D, V] = svd(J, 'econ'); % SVD of the default J is desired
    disp('2) Solution using SVD of default J and multiply with W^(-1/2) (without TRUNCATION)')    
%    J_pinv = W_r * V * D' * inv(D * V' * W_r * W_r * V * D' + dampingFactor * eye(size(D))) * U'
    V_r = W_r * V;
    J_pinv = V_r * D' * inv(D * V_r' * V_r * D' + dampingFactor * eye(size(D))) * U' % WITHOUT TRUNCATION!!!

    [U, singularValuesInv, V] = invSingularValueDecomposition(new_J, dampingFactor); % WITH TRUNCATION!!!
    disp('3) Solution using the new_J = J * W^(-1/2) for SVD (with TRUNCATION)')    
    J_svd_pinv = V * singularValuesInv * U'
    

    q_dot_out = W_r * J_svd_pinv * v_in


    disp('---------- Old solution in augmented solver (WLN_JLA + TRUNCATION) ------------------------') 
    disp('awjlawlkjwakllwka')
% Eigen::JacobiSVD<Eigen::MatrixXd> svdWholeMatrix(jac_.data*W_jla.inverse()*jac_.data.transpose()+Wv,Eigen::ComputeFullU | Eigen::ComputeFullV);
    tmpMatrix = J * inv(W_JLA) * J_T + dampingFactor * eye(size(matProd_J)); 
    [U, D, V] = svd(tmpMatrix)
    [rS, cS] = size(D);
    singularValuesInv = zeros(cS, rS);
    for i=1 : rS    
        if i <= cS
            if D(i,i) <= EPS
                disp('TRUNCATED!!!')
                singularValuesInv(i, i) = 0.0;
            else
                singularValuesInv(i, i) = 1.0 / D(i, i);
            end
        end
    end

    tmp_J_pinv = V * singularValuesInv * U'
    W_r * J_T * tmp_J_pinv
    q_dot_out = inv(W_JLA) * J_T * tmp_J_pinv * v_in
end

% tempSinv = 1 ./ S; does not work for all matrices 
% tempSinv(tempSinv == Inf) = 0.0; 
% singularValuesInv = tempSinv'; 


