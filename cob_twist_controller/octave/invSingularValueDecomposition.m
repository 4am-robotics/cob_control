function [U, singularValuesInv, V] = invSingularValueDecomposition(J, dampingFact)
    global EPS;
    disp('Calculation with SVD');
    [U, S, V] = svd(J, 'econ');
    [rS, cS] = size(S);
    singularValuesInv = zeros(cS, rS);

    if nargin > 1 % in case of damping
        disp('Requested DAMPED solution:')
        for i=1 : rS
            if i <= cS
                if (S(i,i) * S(i,i) + dampingFact) <= EPS
                    singularValuesInv(i, i) = 0.0;
                else
                    singularValuesInv(i, i) = S(i, i) / (S(i,i) * S(i,i) + dampingFact);
                end
            end
        end
    else % no damping
        disp('Requested NORMAL solution:')
        for i=1 : rS
            if i <= cS
                if S(i,i) <= EPS
                    singularValuesInv(i, i) = 0.0;
                else
                    singularValuesInv(i, i) = 1.0 / S(i, i);
                end
            end
        end
    end
end
