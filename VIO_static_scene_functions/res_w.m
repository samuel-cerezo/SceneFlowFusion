function res = res_w(x,delta_t_ij,vel_ang,NoF,i)
% Local Coordinates

bias_gyr = [x(end-5), x(end-4), x(end-3)]';

imu_vel_ang = vel_ang(:,7*(i-1)+1);


w = [x(6*(i-1)+4);x(6*(i-1)+5);x(6*(i-1)+6)];
%Rij = reshape(x(1:9),[3,3]);    %frame j expressed in j coordinates

if (i>1)

    Rij = eye(3);
    for j=2:i
        wi = [x(6*(j-2)+4);x(6*(j-2)+5);x(6*(j-2)+6)];
        Rij = Rij*mapeo_exponencial_SO3(wi*delta_t_ij);
    end

    w = Rij*w;
end

wx = w(1);
wy = w(2);
wz = w(3);


res = [wx,wy,wz]' - ( imu_vel_ang - bias_gyr );

end
