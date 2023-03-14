
% Derivadas que formaran parte del residuo visual

function [Zu,Zv,Zt] = partial_derivates(id0,id1,delta_t)

if (id0 >= 0) && (id0 < 10)
    nombre_escena0 = 'scene_00';
elseif (id0 > 9) && (id0 < 100)
    nombre_escena0 = 'scene_0';
elseif (id0 > 99)
    nombre_escena0 = 'scene_';
end
if (id1 >= 0) && (id1 < 10)
    nombre_escena1 = 'scene_00';
elseif (id1 > 9) && (id1 < 100)
    nombre_escena1 = 'scene_0';
elseif (id1 > 99)
    nombre_escena1 = 'scene_';
end

carpeta_rgb = 'Dataset_ICL_NUIM/traj1_frei_png/rgb/';
carpeta_profundidad = 'Dataset_ICL_NUIM/traj1_frei_png/depth/';
carpeta = 'Dataset_ICL_NUIM/office_room_traj1_loop/';

id0_s = string(id0);
id1_s = string(id1);


K = getcamK(strcat(carpeta,nombre_escena0,id0_s,'.txt'));
fx = K(1,1);
fy = K(2,2);
u0 = K(1,3);
v0 = K(2,3);
u = repmat([1:640],480,1);
v = repmat([1:480]',1,640);
u_u0_by_fx = (u - u0)/fx;
v_v0_by_fy = (v - v0)/fy;
size(u);
size(v);
z0 = load(strcat(carpeta,nombre_escena0,id0_s,'.depth'));
z0 = reshape(z0,640,480)' ;  %z is radial 
z0 = z0 ./ sqrt(u_u0_by_fx.^2 + v_v0_by_fy.^2 + 1);

K = getcamK(strcat(carpeta,nombre_escena1,id1_s,'.txt'));
fx = K(1,1);
fy = K(2,2);
u0 = K(1,3);
v0 = K(2,3);
u_u0_by_fx = (u - u0)/fx;
v_v0_by_fy = (v - v0)/fy;
z1 = load(strcat(carpeta,nombre_escena1,id1_s,'.depth'));
z1 = reshape(z1,640,480)' ;  %z is radial 
z1 = z1 ./ sqrt(u_u0_by_fx.^2 + v_v0_by_fy.^2 + 1);


[Zu,Zv] = gradient(z0);
Zt = (z1-z0)/delta_t;

end
