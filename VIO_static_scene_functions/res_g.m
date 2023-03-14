function res = res_w(x,vel_ang,N)

% N is the number of images

% bias_gyr(1,1) = x(end-14);
% bias_gyr(2,1) = x(end-13);
% bias_gyr(3,1) = x(end-12);

bias_gyr = [x((N*6)+4) x((N*6)+5) x((N*6)+6)]';

error = zeros(1,N);

for i=1:N
        dif = [x(6*(i-1)+4); x(6*(i-1)+5); x(6*(i-1)+6)] - (vel_ang(:,7*(i-1) + 1) - bias_gyr);        
        error(i) = sum(dif.^2);
end

res = sum(error);

end


% bias_gyr(1,1) = x(16);
% bias_gyr(2,1) = x(17);  
% bias_gyr(3,1) = x(18); 
% error1 = [x(4);x(5);x(6)] - (vel_ang(:,1) - bias_gyr);
% error2 = [x(10);x(11);x(12)] - (vel_ang(:,8) - bias_gyr);
%res = (sum(error1.^2)+sum(error2.^2)) ;
