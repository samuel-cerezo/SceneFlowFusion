
% Variable x: [vx0,vy0,vz0,wx0,wy0,wz0,vx1,vy1,vz1,wx1,wy1,wz1, ... ,vx18,vy18,vz18,wx18,wy18,wz18]

function obj = objective_prueba(x,X,Y,Z,Zu,Zv,Zt,alto,ancho,fx,fy)
% 
% addpath(genpath('..'));
% FR = 30;                    %Frame rate = 30Hz
% delta_t = 1/FR;
% carpeta = 'Dataset_ICL_NUIM/office_room_traj1_loop/';
% 
% id0=319;
% 
% 
% id1 = id0+1;
% 
% if (id0 >= 0) && (id0 < 10)
%     nombre_escena0 = 'scene_00';
% elseif (id0 > 9) && (id0 < 100)
%     nombre_escena0 = 'scene_0';
% elseif (id0 > 99)
%     nombre_escena0 = 'scene_';
% end
% id0_s = string(id0);
% 
% %%%%%%%%%%%%%%%%%        3D positions in camera's coordinates       %%%%%%%%%%%%%%%%%%%%%%
% K = getcamK(strcat(carpeta,nombre_escena0,id0_s,'.txt'));
% fx = K(1,1);
% fy = K(2,2);
% u0 = K(1,3);
% v0 = K(2,3);
% u = repmat([1:640],480,1);
% v = repmat([1:480]',1,640);
% u_u0_by_fx = (u - u0)/fx;
% v_v0_by_fy = (v - v0)/fy;
% size(u);
% size(v);
% Z = load(strcat(carpeta,nombre_escena0,id0_s,'.depth'));
% Z = reshape(Z,640,480)' ;  %z is radial 
% Z = Z ./ sqrt(u_u0_by_fx.^2 + v_v0_by_fy.^2 + 1);
% X = ((u-u0)/fx).*Z;
% Y = ((v-v0)/fy).*Z;
% 
% [Zu,Zv,Zt] = partial_derivates(id0,id1,delta_t);
% 
% dim = size(Z);
% alto = dim(1);
% ancho = dim(2);


obj = sum(sum(((1*ones(alto,ancho) + Zu.*((X*fx)./(Z.^2)) + Zv.*((Y*fy)./(Z.^2))).*(x(3)*ones(alto,ancho) + Y*x(4) - X*x(5)) + (fx)*(Zu./Z).*(-x(1)*ones(alto,ancho) + Y*x(6) - Z*x(5)) + (fy)*(Zv./Z).*(-x(3)*ones(alto,ancho) - X*x(6) + Z*x(4)) + Zt).^2));




end
