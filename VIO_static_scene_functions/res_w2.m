function res = res_w2(x,delta_t_ij,initial_bias,NoF,i)
% Local Coordinates
% initial_bias -> 3x1 vector

bias_gyr = [x(end-5), x(end-4), x(end-3)]';

res = initial_bias - bias_gyr;

end