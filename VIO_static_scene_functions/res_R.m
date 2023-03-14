function res = res_R(x,i,j,vel_ang_ext,delta_t_ij,nro_intervalos_u)

%  (res_V(x,1,8,w_aprox_ext,a_aprox_ext,delta_t,nro_intervalos_u,R_ini,g_direction,pair,1))


delta_t = delta_t_ij/nro_intervalos_u;

bias_gyr = repmat([x(end-5), x(end-4), x(end-3)]',1,8);
delta_bias_g = [0,0,0]';  %considero que el bias no cambia del instante "i" al "j".

res = zeros(1,3);

l=1;
        vel_ang=vel_ang_ext(:,(7*(l-1) + 1) :(7*(l-1) + 8));
%        Ri = R_i{l};
        deriv_deltaR = eye(3);   %el resultado es una matriz de 3x3 formada por vectores ortonormales
        for k=1:(j-i)
            deriv_deltaR = deriv_deltaR - (deltaR_monio(k+1,j,vel_ang,bias_gyr,delta_t))'*(Jr_SO3((vel_ang(:,k) - bias_gyr(:,k))*delta_t))*delta_t;
        end

        delta_R = mapeo_exponencial_SO3([x(6*(l-1)+4); x(6*(l-1)+5); x(6*(l-1)+6)]*delta_t_ij);
        %R_j = R_i*delta_R;
        
        R_i = eye(3,3);
        R_j = reshape(x(1:9),[3,3]);
        residuoR_SO3 = ((deltaR_monio(i,j,vel_ang,bias_gyr,delta_t)*mapeo_exponencial_SO3(deriv_deltaR*delta_bias_g))')*R_i'*R_j;
        
        % Logaritmic mapping here
        theta_residuoR = acos(0.5*(trace(residuoR_SO3)-1));
        residuoR_so3 = (theta_residuoR / (2*sin(theta_residuoR)))*(residuoR_SO3 - residuoR_SO3');
        residuoR_RE3 = [-residuoR_so3(2,3); residuoR_so3(1,3); -residuoR_so3(1,2)];       
        % Column vector (3x1)
        %residual(l) = sum(residuoR_RE3.^2);

%for cicle

%res = sum(residual);
res = residuoR_RE3;
end
