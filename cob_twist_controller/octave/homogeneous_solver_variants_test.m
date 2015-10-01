function homogeneous_solver_test(m,n)
  J = rand(m,n)
  %v=rand(m,1)
  v=zeros(m,1)
  q_0 = rand(n,1)
  
  r=rank(J)
  [U,S,V]=svd(J);
  I = eye(n,n);
  S_pinv=zeros(n,m);
  S_pinv_dls=zeros(n,m);
  
  w_threshold = 0.05;
  lambda_max = 0.5;
  manip_meas = sqrt(abs(det(J * J')))
  factor = 0.0;
  if manip_meas < w_threshold
    tmp_w = (1 - manip_meas / w_threshold);
    factor = lambda_max * tmp_w * tmp_w 
  endif
  
  for k=1:r
    S_pinv(k,k)=1/S(k,k);
    S_pinv_dls(k,k)= S(k,k) / (S(k,k) * S(k,k) + factor * factor);
  end
  S_pinv;
  S_pinv_dls;
  J_pinv = V*S_pinv*transpose(U);
  J_pinv_dls = V*S_pinv_dls*transpose(U);

  % homogeneous solutions
  q_sol_hom1 = J_pinv*v %least-squares
  v_hom1 = J*q_sol_hom1
  error_v = v - v_hom1
  q_sol_hom2 = J_pinv_dls*v %damped-least-squares
  v_hom2 = J*q_sol_hom2
  error_v = v - v_hom2

  % particular solutions (gpm)
  P1=I-J_pinv*J;
  P2=I-J_pinv_dls*J;
  disp("--------- LS-LS ---------");
  q_sol_gpm1 = J_pinv*v + P1*q_0 %ls - ls
  v_gpm1 = J*(q_sol_gpm1+P1*q_0)
  error_v = v - v_gpm1
  error_v_hom1 = v_hom1 - v_gpm1
  error_v_hom2 = v_hom2 - v_gpm1
  disp("--------- LS-DLS ---------");
  q_sol_gpm2 = J_pinv*v + P2*q_0 %ls - damped
  v_gpm2 = J*(q_sol_gpm2+P2*q_0)
  error_v = v - v_gpm2
  error_v_hom1 = v_hom1 - v_gpm2
  error_v_hom2 = v_hom2 - v_gpm2
  disp("--------- DLS-LS ---------");
  q_sol_gpm3 = J_pinv_dls*v + P1*q_0 %damped - ls
  v_gpm3 = J*(q_sol_gpm3+P1*q_0)
  error_v = v - v_gpm3
  error_v_hom1 = v_hom1 - v_gpm3
  error_v_hom2 = v_hom2 - v_gpm3
  disp("--------- DLS-DLS ---------");
  q_sol_gpm4 = J_pinv_dls*v + P2*q_0 %damped - damped
  v_gpm4 = J*(q_sol_gpm4+P2*q_0)
  error_v = v - v_gpm4
  error_v_hom1 = v_hom1 - v_gpm4
  error_v_hom2 = v_hom2 - v_gpm4

end
