function homogeneous_solver_test(m,n)
  J = rand(m,n)
  r=rank(J)
  [U,S,V]=svd(J)
  I = eye(n,n);
  S_pinv=zeros(n,m);
  for k=1:r
    S_pinv(k,k)=1/S(k,k);
  end
  S_pinv
  J_pinv = V*S_pinv*transpose(U)
  P=I-J_pinv*J
  
  v=rand(m,1)
  
  q_sol = J_pinv*v
  v_test = J*q_sol
  
  q_0 = rand(n,1)
  v_test2 = J*(q_sol+P*q_0)
  
  q_sol_test = J_pinv*v + (I-J_pinv*J)*q_0
  
  error_v = v_test - v_test2
  error_q = q_sol - q_sol_test
end
