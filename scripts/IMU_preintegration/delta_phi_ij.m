% Esta funcion sirve para obtener los valores de preintegración del ruido,
% segun la formula (42) del paper ON MANIFOLD PREINTEGRATION ...

%                   ---------- Variables ---------

function [delta_phi] = delta_phi_ij(i,j,vel_ang,bias_gyr,eta_gyr,delta_t)    %Funcion corroborada OK
    
    if (j<i)
        error('Error. El índice j debe ser mayor o igual que i')
    end
    
    delta_phi = zeros(3,1);   %el resultado es un vector columna de 3 elementos
    
    for k=1:(j-i)
        delta_phi = delta_phi + transpose(deltaRgorro_ij(k+1,j,vel_ang,bias_gyr,delta_t))*Jr_SO3((vel_ang(:,k) - bias_gyr(:,k))*delta_t)*eta_gyr(:,k)*delta_t;   
    end
end

% delta_phi es un vector columna de 3 elementos (3x1)

