% Esta funcion sirve para obtener los valores de preintegración de IMU. 

% NOTA: Esta función supone conocidos los valores del bias y del ruido "eta" en
% cada instante.

%                   ---------- Variables ---------
%   "i" es entero, indica el indice de comienzo
%   "j" es entero, indica el indice del fin
%   "acel_lin" es una matriz de (j-i+1) columnas, una por cada instante de tiempo "k": [acel_lin(1:3,i), acel_lin(1:3,i+1), ... , acel_lin(1:3,j-1)]
%   "bias_ace" es una matriz de (j-i+1) columnas, una por cada instante de tiempo "k": [bias_ace(1:3,i), bias_ace(1:3,i+1), ... , bias_ace(1:3,j-1)]
%   "eta_ace" es una matriz de (j-i+1) columnas, una por cada instante de tiempo "k": [eta_ace(i), eta_ace(i+1), ... , eta_ace(j-1)]
%   "vel_ang" es una matriz de (j-i+1) columnas, una por cada instante de tiempo "k": [vel_ang(1:3,i), vel_ang(1:3,i+1), ... , vel_ang(1:3,j-1)]
%   "bias_gyr" es una matriz de (j-i+1) columnas, una por cada instante de tiempo "k": [bias_gyr(1:3,i), bias_gyr(1:3,i+1), ... , bias_gyr(1:3,j-1)]
%   "eta_gyr" es una matriz de (j-i+1) columnas, una por cada instante de tiempo "k": [eta_gyr(i), eta_gyr(i+1), ... , eta_gyr(j-1)]
%   "delta_t" es un numero indicando el espacio temporal entre dos instantes de tiempo (NO es tj-ti!!)

function [delta_P] = deltaP_ij(i,j,acel_lin,bias_ace,eta_ace,vel_ang,bias_gyr,eta_gyr,delta_t)  %Funcion corroborada OK
    
    if (j<i)
        error('Error. El índice j debe ser mayor o igual que i')
    end
    
    delta_P = zeros(3,1);   %el resultado es un vector columna de 3 elementos
    
    for k=1:(j-i)
        delta_P = delta_P + deltaV_ij(1,k,acel_lin,bias_ace,eta_ace,vel_ang,bias_gyr,eta_gyr,delta_t)*delta_t + (1/2)*deltaR_ij(1,k,vel_ang,bias_gyr,eta_gyr,delta_t)*(acel_lin(:,k) - bias_ace(:,k) - eta_ace(:,k))*(delta_t^2);
    end
end
