function homogeneous_solver_test(m,n)
  disp("--------- DLS solution !!! ----------------");
  J = rand(m,n)
  r=rank(J)
  [U,S,V]=svd(J);
  I = eye(n,n);
  S_pinv=zeros(n,m);
  
  w_threshold = 0.05;
  lambda_max = 0.5;
  manip_meas = sqrt(abs(det(J * J')))
  factor = 0.0;
  if manip_meas < w_threshold
    tmp_w = (1 - manip_meas / w_threshold);
    factor = lambda_max * tmp_w * tmp_w 
  endif
  
  for k=1:r
    S_pinv(k,k)= S(k,k) / (S(k,k) * S(k,k) + factor * factor);
  end
  S_pinv;
  J_pinv = V*S_pinv*transpose(U);
  P=I-J_pinv*J;

  v=rand(m,1)

  q_sol_hom = J_pinv*v;
  v_hom = J*q_sol_hom

  q_0 = rand(n,1)
  q_sol_gpm = J_pinv*v + P*q_0;
  v_gpm = J*(q_sol_gpm+P*q_0)

  error_v = v_hom - v_gpm
  error_q = q_sol_hom - q_sol_gpm
end
