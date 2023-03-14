% entrada:
i=1;
j=8;
delta_bias_g = [0,0,0]';  %considero que el bias no cambia del instante "i" al "j".
delta_bias_a = [0,0,0]';  %considero que el bias no cambia del instante "i" al "j".

%   "delta_t" es un numero indicando el espacio temporal entre dos instantes de medicion de IMU(NO es tj-ti!!)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                           RESIDUO EN R
% ------------------ Calculo de derivada parcial ---------------------

deriv_deltaR = eye(3);   %el resultado es una matriz de 3x3 formada por vectores ortonormales

    for k=1:(j-i)
        deriv_deltaR = deriv_deltaR - (deltaR_monio(k+1,j,vel_ang,bias_gyr,delta_t))'*(Jr_SO3((vel_ang(:,k) - bias_gyr(:,k))*delta_t))*delta_t;
    end
% --------------------------------------------------------------------
residuoR = (deltaR_monio(k+1,j,vel_ang,bias_gyr,delta_t)*mapeo_exponencial_SO3(deriv_deltaR*delta_bias_g))'*R_i'*R_j;








