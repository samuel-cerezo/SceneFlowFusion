% entrada:
i=1;
j=8;
delta_bias_g = [0,0,0]';  %considero que el bias no cambia del instante "i" al "j".
delta_bias_a = [0,0,0]';  %considero que el bias no cambia del instante "i" al "j".


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                   RESIDUO EN R
deriv_deltaR = eye(3);   %el resultado es una matriz de 3x3 formada por vectores ortonormales
    for k=1:(j-i)
        deriv_deltaR = deriv_deltaR - (deltaR_monio(k+1,j,vel_ang,bias_gyr,delta_t))'*(Jr_SO3((vel_ang(:,k) - bias_gyr(:,k))*delta_t))*delta_t;
    end
residuoR_SO3 = ((deltaR_monio(k+1,j,vel_ang,bias_gyr,delta_t)*mapeo_exponencial_SO3(deriv_deltaR*delta_bias_g))')*R_i'*R_j;
theta_residuoR = acos(0.5*(trace(residuoR_SO3)-1));
residuoR_so3 = (theta_residuoR / (2*sin(theta_residuoR)))*(residuoR_SO3 - residuoR_SO3');
residuoR_RE3 = [-residuoR_so3(2,3), residuoR_so3(1,3), -residuoR_so3(1,2)]';       


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                           RESIDUO EN V

deriv_deltaV_a = eye(3);   %el resultado es una matriz de 3x3 formada por vectores ortonormales
deriv_deltaV_a = eye(3);

    for k=1:(j-i)
        deriv_deltaV_a = deriv_deltaV_a - (deltaR_monio(i,k,vel_ang,bias_gyr,delta_t)*delta_t);
    end

    
    for k=1:(j-i)
        aux = acel_lin(:,k) - bias_ace(:,k);
        aux_gorro = [0,-aux(3),aux(2);aux(3),0,-aux(1);-aux(2),aux(1),0];
        deriv_deltaV_g = deriv_deltaV_g - (deltaR_monio(i,k,vel_ang,bias_gyr,delta_t)*aux_gorro*deriv_deltaR*delta_t);
    end
    
    
    for k=1:(j-i) 
        aux = acel_lin(:,k) - bias_ace(:,k);
        aux_gorro = [0,-aux(3),aux(2);aux(3),0,-aux(1);-aux(2),aux(1),0];
        deriv_deltaP_a = deriv_deltaP_a + (deriv_deltaV_a*delta_t - 0.5*deltaR_monio(i,k,vel_ang,bias_gyr,delta_t)*(delta_t^2));
    end


    for k=1:(j-i) 
        aux = acel_lin(:,k) - bias_ace(:,k);
        aux_gorro = [0,-aux(3),aux(2);aux(3),0,-aux(1);-aux(2),aux(1),0];
        deriv_deltaP_g = deriv_deltaP_g + (deriv_deltaV_g*delta_t - 0.5*deltaR_monio(i,k,vel_ang,bias_gyr,delta_t)*aux_gorro*deriv_deltaR*(delta_t^2));
    end

    
residuoP_RE3  = R_i'*(P_j - P_i - 0.5*vector_g*((delta_t*nro_intervalos)^2) - V_i*delta_t*nro_intervalos) - (deltaP_monio(i,j,acel_lin,bias_ace,vel_ang,bias_gyr,delta_t) + deriv_deltaP_g*delta_bias_g + deriv_deltaP_a*delta_bias_a);

residuoV_RE3 = R_i'*(V_j - V_i - vector_g*(delta_t*nro_intervalos) - V_i*delta_t*nro_intervalos) - (deltaV_monio(i,j,acel_lin,bias_ace,vel_ang,bias_gyr,delta_t) + deriv_deltaV_g*delta_bias_g + deriv_deltaV_a*delta_bias_a);


