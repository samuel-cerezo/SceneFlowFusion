function res = res_cov(x,i,j,IMU_gyr,IMU_ace,delta_t_ij,nro_intervalos_u,Ri,N,gi,Sigma_ij,id0)

%id0=319;
Nimu = 2;
w_aprox_i = [IMU_gyr(1,(((id0-1)*7)+1):(((id0-1)*7)+((Nimu-1)*7+1)));IMU_gyr(2,(((id0-1)*7)+1):(((id0-1)*7)+((Nimu-1)*7+1)));IMU_gyr(3,(((id0-1)*7)+1):(((id0-1)*7)+((Nimu-1)*7+1)))];
a_aprox_i = [IMU_ace(1,((id0*7)+1):((id0*7)+((Nimu-1)*7+1)));IMU_ace(2,((id0*7)+1):((id0*7)+((Nimu-1)*7+1)));IMU_ace(3,((id0*7)+1):((id0*7)+((Nimu-1)*7+1)))];

id0=id0+1;
w_aprox_i_plus_1 = [IMU_gyr(1,(((id0-1)*7)+1):(((id0-1)*7)+((Nimu-1)*7+1)));IMU_gyr(2,(((id0-1)*7)+1):(((id0-1)*7)+((Nimu-1)*7+1)));IMU_gyr(3,(((id0-1)*7)+1):(((id0-1)*7)+((Nimu-1)*7+1)))];
a_aprox_i_plus_1 = [IMU_ace(1,((id0*7)+1):((id0*7)+((Nimu-1)*7+1)));IMU_ace(2,((id0*7)+1):((id0*7)+((Nimu-1)*7+1)));IMU_ace(3,((id0*7)+1):((id0*7)+((Nimu-1)*7+1)))];

%xi = [x(1:6) x(13:21)];
xj = x;
%res_i = res_V(xi,i,j,w_aprox_i,a_aprox_i,delta_t_ij,nro_intervalos_u,Ri,N,gi);
res_i_plus_1 = res_V_cov(xj,i,j,w_aprox_i_plus_1,a_aprox_i_plus_1,delta_t_ij,nro_intervalos_u,Ri,N,gi);


%est_xi = vect_x_igorro{1,1};
%est_xj = vect_x_igorro{2,1}(7:21);
%delta_x_i = (xi-est_xi);
%delta_x_j = (xj-est_xj);

res = (res_i_plus_1')*(inv(Sigma_ij))*(res_i_plus_1);

end
