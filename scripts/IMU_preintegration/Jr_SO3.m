
function J = Jr_SO3(phi)

%   "phi" es un vector columna de 3 elementos
%   J es una matriz de 3x3.


%   J es el jacobiano de S03 y relaciona los incrementos aditivos del
%   espacio tangente(grupo de LIE), con los incrementos multiplicativos del
%   manifold (SO3).


theta=norm(phi);
phi_gorro = phi(1)*[0,0,0;0,0,-1;0,1,0] + phi(2)*[0,0,1;0,0,0;-1,0,0] + phi(3)*[0,-1,0;1,0,0;0,0,0];

% wx = A(1:3,1:3);
% t = A(1:3,4);
% w =   [-wx(2,3),wx(1,3),-wx(1,2)]';  %% <---- primer componente

if theta ~= 0
    J = eye(3,3) - ((1 - cos(theta))/(theta^2))*phi_gorro  + ((theta - sin(theta))/(theta^3))*(phi_gorro^2);    
else
    J = eye(3,3);
end


