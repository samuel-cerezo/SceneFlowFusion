% Esta funcion sirve para obtener los valores de preintegración de IMU. 
% Obtiene la "evolución" de la matriz de rotación R, entre dos instantes 
% t_i y t_j (sin considerar el efecto del bias del giróscopo.


%   ∆R = Ri'*Rj 



% NOTA: Esta función supone conocidos los valores del bias y del ruido "eta" en
% cada instante.

%                   ---------- Variables ---------
%   "i" es entero, indica el indice de comienzo
%   "j" es entero, indica el indice del fin
%   "vel_ang" es una matriz de (j-i+1) columnas, una por cada instante de tiempo "k": [vel_ang(1:3,i), vel_ang(1:3,i+1), ... , vel_ang(1:3,j-1)]
%   "bias_gyr" es una matriz de (j-i+1) columnas, una por cada instante de tiempo "k": [bias_gyr(1:3,i), bias_gyr(1:3,i+1), ... , bias_gyr(1:3,j-1)]
%   "eta_gyr" es una matriz de (j-i+1) columna, una por cada instante de tiempo "k": [eta_gyr(i), eta_gyr(i+1), ... , eta_gyr(j-1)]
%   "delta_t" es un numero indicando el espacio temporal entre dos instantes de medicion de IMU(NO es tj-ti!!)

function [delta_R] = deltaR_monio(i,j,vel_ang,bias_gyr,delta_t)    %Funcion corroborada OK
    
    if (j<i)
        error('Error. El índice j debe ser mayor o igual que i')
    end
    delta_R = eye(3);   %el resultado es una matriz de 3x3 formada por vectores ortonormales
    for k=1:(j-i)
        delta_R = delta_R*mapeo_exponencial_SO3((vel_ang(:,k) - bias_gyr(:,k))*delta_t);
    end
    
    
   %      La siguiente manera seria separando el ruido del producto (formula (35) del paper ON MANIFOLD)
   % delta_R = deltaRgorro_ij(i,j,vel_ang,bias_gyr,delta_t)*mapeo_exponencial_SO3(-delta_phi_ij(i,j,vel_ang,bias_gyr,eta_gyr,delta_t));
end
