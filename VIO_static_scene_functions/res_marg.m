function res = res_marg(x,vect_x_igorro,H_marg)

est_xi = vect_x_igorro{1,1};
est_xj = vect_x_igorro{2,1}(7:21);
xi = [x(1:6) x(13:21)];
xj = x(7:21);

delta_x_i = (xi-est_xi);
delta_x_j = (xj-est_xj);

res = (delta_x_j)*(H_marg)*(delta_x_i)';

end
