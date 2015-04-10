function [] = svd_pseudoinverse(n, m, dampingFactor)
    clc; 

    global EPS;
    EPS = 1.0e-12 
    
    if nargin < 3
        dampingFactor = 0.0
    endif 

    % J = [1 -1 0; 0 1 1];
    J = [2 3 5; 1 7 4];
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
%    disp('Calculation with SVD');
%    [U, S, V] = svd(J)
%    [rS, cS] = size(S);
%    singularValuesInv = zeros(cS, rS);
%    for i=1 : rS    
%        if i <= cS
%            if S(i,i) <= EPS
%                singularValuesInv(i, i) = 0.0;
%            else
%                singularValuesInv(i, i) = 1.0 / S(i, i);
%            end
%        end
%    end







    J_svd_pinv = V * singularValuesInv * U'; 
    disp('Result: ');
    J_svd_pinv



    % ---------- SR-Inverse with damping -------------------------------------
    disp('---------- SR-Inverse with damping ------------------------')    
    k = 55.0; % damping factor 
    % J = [1 2 3 4; 5 6 7 8]; 
    J = [2 3 5; 1 7 4];
    J_T = J'; 

    tmp = J * J_T; 

    W = k * eye(size(tmp));

    disp('Damped Result: ');
    J_pinv = J_T * inv(tmp + W)

    [U, singularValuesInv, V] = invSingularValueDecomposition(J, k); 
    J_svd_pinv = V * singularValuesInv * U'; 
    disp('Damped Result: ');
    J_svd_pinv


    % ---------- SR-Inverse with Weighting / JLA -------------------------------------

    disp('---------- SR-Inverse with Weighting / JLA ------------------------')    
    k = 55.0; % damping factor 
    % J = [1 2 3 4; 5 6 7 8]; 
    J = [2 3 5; 1 7 4];
    J_T = J'; 

    [rowsJ, colsJ] = size(J);
    w_jla = ones(colsJ, 1); 
    for i = 1 : colsJ
        w_jla(i) = i * i; 
    end    
    
    W_JLA = diag(w_jla);
    
    half_inv_W_JLA = W_JLA^(-1/2); % weighting affects Jacobian
    new_J = J * half_inv_W_JLA; 

    tmp = J * inv(W_JLA) * J_T;

    W = k * eye(size(tmp)); % damping

    disp('Damped Result: ');
    J_pinv = inv(W_JLA) * J_T * inv(tmp + W)
    
    [U, singularValuesInv, V] = invSingularValueDecomposition(tmp, k); 
    J_svd_pinv = V * singularValuesInv * U'; 
    disp('WRONG Damped Result by SVD (with new_J = J * inv(W_JLA) * J_T): ');
    J_svd_pinv % omitted W_JLA and J_T

    disp('Corrected (anyway WRONG) Damped Result by SVD (with new_J = J * inv(W_JLA) * J_T): ');
    test = inv(W_JLA) * J_T * J_svd_pinv % recognized W_JLA and J_T

    [U, singularValuesInv, V] = invSingularValueDecomposition(new_J, k); 
    J_svd_pinv = V * singularValuesInv * U'; 
    disp('Initially correct damped Result by SVD (with adapted J -> new_J): ');
    J_svd_pinv % recognized W_JLA and J_T
        
    half_inv_W_JLA * J_svd_pinv
end

% tempSinv = 1 ./ S; does not work for all matrices 
% tempSinv(tempSinv == Inf) = 0.0; 
% singularValuesInv = tempSinv'; 


