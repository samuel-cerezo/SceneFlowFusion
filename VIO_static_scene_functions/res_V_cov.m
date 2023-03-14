function res = res_V_cov(x,i,j,vel_ang_ext,acel_lin_ext,delta_t_ij,nro_intervalos_u,Ri,N,gi)

delta_t = delta_t_ij/nro_intervalos_u;

bias_gyr = repmat([x((N*6)+4) x((N*6)+5) x((N*6)+6)]',1,8);

bias_ace = repmat([x((N*6)+7) x((N*6)+8) x((N*6)+9)]',1,8);

% ----  Gravity vector parametrization
%
delta = [x((N*6)+1);x((N*6)+2)];
xi=(gi(1));
yi=(gi(2));
zi=(gi(3));
exp = [cos(norm(delta)) ; sinc(norm(delta))*delta ];
r = sqrt(yi^2 + zi^2);
alfa = atan2(zi,yi);
expR = [xi -r 0 ;...
     yi xi*cos(alfa) -sin(alfa) ;...
     zi xi*sin(alfa) cos(alfa)];

g_direction = expR*exp;
g_vector = -981*g_direction;

% -------------------------------------------

%g_vector = -100*[x((N*6)+1);x((N*6)+2);x((N*6)+3)];
%g_vector = [x(end-8);x(end-7);x(end-6)];

delta_bias_g = [0,0,0]';  %considero que el bias no cambia del instante "i" al "j".
delta_bias_a = [0,0,0]';
deriv_deltaR = eye(3);   %el resultado es una matriz de 3x3 formada por vectores ortonormales
deriv_deltaV_a = zeros(3,3);   %el resultado es una matriz de 3x3 formada por vectores ortonormales
deriv_deltaV_g = zeros(3,3);

residual = zeros(1,(N-1));

    for l=1:(N-1)
        V_i = [x(6*(l-1)+1);x(6*(l-1)+2);x(6*(l-1)+3)];
        V_j = [x(6*(l-1)+7);x(6*(l-1)+8);x(6*(l-1)+9)];

        vel_ang=vel_ang_ext(:,(7*(l-1) + 1) :(7*(l-1) + 8));
        acel_lin=acel_lin_ext(:,(7*(l-1) + 1) :(7*(l-1) + 8));
        %Ri = R_i{l};
        
            for k=1:(j-i)
                deriv_deltaR = deriv_deltaR - (deltaR_monio(k+1,j,vel_ang,bias_gyr,delta_t))'*(Jr_SO3((vel_ang(:,k) - bias_gyr(:,k))*delta_t))*delta_t;
            end
            for k=1:(j-i)
                deriv_deltaV_a = deriv_deltaV_a - (deltaR_monio(i,k,vel_ang,bias_gyr,delta_t)*delta_t);
            end
            for k=1:(j-i)
                aux = acel_lin(:,k) - bias_ace(:,k);
                aux_gorro = [0,-aux(3),aux(2);aux(3),0,-aux(1);-aux(2),aux(1),0];
                deriv_deltaV_g = deriv_deltaV_g - (deltaR_monio(i,k,vel_ang,bias_gyr,delta_t)*aux_gorro*deriv_deltaR*delta_t);
            end

        %residuoV_RE3 = R_i'*(V_j - V_i + g_vector*(delta_t*nro_intervalos_u) - V_i*delta_t*nro_intervalos_u) - (deltaV_monio(i,j,acel_lin,bias_ace,vel_ang,bias_gyr,delta_t) + deriv_deltaV_g*delta_bias_g + deriv_deltaV_a*delta_bias_a);
        residuoV_RE3 = Ri'*(V_j - V_i - g_vector*(delta_t*nro_intervalos_u)) - (deltaV_monio(i,j,acel_lin,bias_ace,vel_ang,bias_gyr,delta_t) + 0 + 0);
        
        %residual(l) = sum(residuoV_RE3.^2);
    end

res = residuoV_RE3;

end

% El resultado es un vector de 3x1
