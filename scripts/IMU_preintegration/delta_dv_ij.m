% Esta funcion sirve para obtener los valores de preintegración del ruido,
% segun la formula (42) del paper ON MANIFOLD PREINTEGRATION ...

%                   ---------- Variables ---------

function [delta_v] = delta_dv_ij(i,j,acel_lin,bias_ace,eta_ace,vel_ang,bias_gyr,eta_gyr,delta_t)    %Funcion corroborada OK
    
    if (j<i)
        error('Error. El índice j debe ser mayor o igual que i')
    end
    
    delta_v = zeros(3,1);   %el resultado es un vector columna de 3 elementos
    
    for k=1:(j-i)
        aux_gorro = (acel_lin(1,k) - bias_ace(1,k))*[0,0,0;0,0,-1;0,1,0] + (acel_lin(2,k) - bias_ace(2,k))*[0,0,1;0,0,0;-1,0,0] + (acel_lin(3,k) - bias_ace(3,k))*[0,-1,0;1,0,0;0,0,0];
        delta_v = delta_v - deltaRgorro_ij(1,k,vel_ang,bias_gyr,delta_t)*aux_gorro*delta_phi_ij(1,k,vel_ang,bias_gyr,eta_gyr,delta_t)*delta_t + deltaRgorro_ij(1,k,vel_ang,bias_gyr,delta_t)*eta_ace(:,k)*delta_t;   
    end
end

% delta_phi es un vector columna de 3 elementos (3x1)
