function res = res_a(x,delta_t_ij,initial_bias,NoF,i)
% Local Coordinates
% initial_bias -> 3x1 vector

bias_acc = [x(end-2), x(end-1), x(end)]';

res = initial_bias - bias_acc;

end
