
%    iteration of preintegrated measurement covariance

% Esta funcion sirve para obtener los valores de covarianza al obtener una nueva medicion de la IMU. 

%                   ---------- Variables ---------
%   "i" es entero, indica el indice de comienzo
%   "j" es entero, indica el indice del fin
%   "vel_ang" es una matriz de (j-i+1) columnas, una por cada instante de tiempo "k": [vel_ang(1:3,i), vel_ang(1:3,i+1), ... , vel_ang(1:3,j-1)]
%   "bias_gyr" es una matriz de (j-i+1) columnas, una por cada instante de tiempo "k": [bias_gyr(1:3,i), bias_gyr(1:3,i+1), ... , bias_gyr(1:3,j-1)]
%   "eta_gyr" es una matriz de (j-i+1) columna, una por cada instante de tiempo "k": [eta_gyr(i), eta_gyr(i+1), ... , eta_gyr(j-1)]
%   "delta_t" es un numero indicando el espacio temporal entre dos instantes de medicion de IMU(NO es tj-ti!!)

%   "cov_eta"(6x6) contiene la covarianza proveniente de las especificaciones de la IMU

function [covariance] = preint_cov(i,j,vel_ang,bias_gyr,acel_lin,bias_ace,cov_ant,delta_t)    
    
    if (j<i)
        error('Error. El índice j debe ser mayor o igual que i')
    end

    % ---------------- Matriz de covarianza del sensor -----------------
    %   Parametros de IMU:
    gyr_noise_density = 0.00012;          %rad/s/sqrt(Hz)
    acc_noise_density = 0.008;            %m/s2/sqrt(Hz)
    freq = 210;   %Muestreo de datos en IMU ---> sensor bandwidth
    var_gyr = gyr_noise_density * sqrt(freq);
    var_acc = acc_noise_density * sqrt(freq);
    cov_eta = [eye(3,3)*var_gyr zeros(3,3); ...
                 zeros(3,3) eye(3,3)*var_acc];     %funciona OK
    % -----------------------------------------------------------------

    A = zeros(9,9);
    A(1:3,1:3) = deltaR_monio(j-1,j,vel_ang,bias_gyr,delta_t)';
    gorro = (acel_lin(1,j-1) - bias_ace(1,i))*[0,0,0;0,0,-1;0,1,0] + (acel_lin(2,j-1) - bias_ace(2,i))*[0,0,1;0,0,0;-1,0,0] + (acel_lin(3,j-1) - bias_ace(3,i))*[0,-1,0;1,0,0;0,0,0];
    A(4:6,1:3) = -deltaR_monio(i,j-1,vel_ang,bias_gyr,delta_t)*gorro*delta_t;
    A(4:6,4:6) = eye(3,3);
    A(7:9,1:3) = -(1/2)*deltaR_monio(i,j-1,vel_ang,bias_gyr,delta_t)*gorro*(delta_t^2);
    A(7:9,4:6) = eye(3,3)*delta_t;
    A(7:9,7:9) = eye(3,3);
    
    B = zeros(9,6);
    B(1:3,1:3) = delta_t*Jr_SO3((vel_ang(:,j-1) - bias_gyr(:,i))*delta_t);
    B(4:6,4:6) = deltaR_monio(i,j-1,vel_ang,bias_gyr,delta_t)*delta_t;
    B(7:9,4:6) = (1/2)*deltaR_monio(i,j-1,vel_ang,bias_gyr,delta_t)*(delta_t^2);
    
    covariance = A*cov_ant*A' + B*cov_eta*B';   %evolucion de la covarianza

end





