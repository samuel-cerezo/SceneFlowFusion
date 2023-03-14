function res = res_V(x,i,j,vel_ang_ext,acel_lin_ext,delta_t_ij,nro_intervalos_u,Ri,gi,NoF,actual_frame)


delta_t = delta_t_ij/nro_intervalos_u;

bias_gyr = repmat([x(end-5), x(end-4), x(end-3)]',1,8);
bias_ace = repmat([x(end-2), x(end-1), x(end)]',1,8);


% ----  Gravity vector parametrization
%delta = [x((NoF*6)+1);x((NoF*6)+2)];

delta = [x(end-7);x(end-6)];

xi=(gi(1));
yi=(gi(2));
zi=(gi(3));
exp = [cos(norm(delta)) ; sinc(norm(delta))*delta ];
r = sqrt(yi^2 + zi^2);
alfa = atan2(zi,yi);
expR = [xi -r 0 ;...
     yi xi*cos(alfa) -sin(alfa) ;...
     zi xi*sin(alfa) cos(alfa)];

g_direction = expR*exp;
g_vector = -981*g_direction;
% -------------------------------------------

% delta_bias_g = [0,0,0]';  %considero que el bias no cambia del instante "i" al "j".
% delta_bias_a = [0,0,0]';
% deriv_deltaR = eye(3);   %el resultado es una matriz de 3x3 formada por vectores ortonormales
% deriv_deltaV_a = zeros(3,3);   %el resultado es una matriz de 3x3 formada por vectores ortonormales
% deriv_deltaV_g = zeros(3,3);

% OBS: Haciendo coincidir las coord del mundo con las de la cámara "i",
% entonces...
%V_i = [x(end-((NoF-actual_frame+1)*6)+8-1);x(end-((NoF-actual_frame+1)*6)+8-2);x(end-((NoF-actual_frame+1)*6)+8-3)];

V_i = [x(6*(actual_frame-1)+1);x(6*(actual_frame-1)+2);x(6*(actual_frame-1)+3)];
V_j = [x(6*(actual_frame-1)+7);x(6*(actual_frame-1)+8);x(6*(actual_frame-1)+9)];

% V_i = [x(6*(actual_frame-1)+1);x(6*(actual_frame-1)+2);x(6*(actual_frame-1)+3)];
% V_j = [x(6*(actual_frame-1)+7);x(6*(actual_frame-1)+8);x(6*(actual_frame-1)+9)];

vel_ang=vel_ang_ext(:,(7*(actual_frame-1) + 1) :(7*(actual_frame-1) + 8));
acel_lin=acel_lin_ext(:,(7*(actual_frame-1) + 1) :(7*(actual_frame-1) + 8));

% for k=1:(j-i)
%     deriv_deltaR = deriv_deltaR - (deltaR_monio(k+1,j,vel_ang,bias_gyr,delta_t))'*(Jr_SO3((vel_ang(:,k) - bias_gyr(:,k))*delta_t))*delta_t;
% end
% for k=1:(j-i)
%     deriv_deltaV_a = deriv_deltaV_a - (deltaR_monio(i,k,vel_ang,bias_gyr,delta_t)*delta_t);
% end
% for k=1:(j-i)
%     aux = acel_lin(:,k) - bias_ace(:,k);
%     aux_gorro = [0,-aux(3),aux(2);aux(3),0,-aux(1);-aux(2),aux(1),0];
%     deriv_deltaV_g = deriv_deltaV_g - (deltaR_monio(i,k,vel_ang,bias_gyr,delta_t)*aux_gorro*deriv_deltaR*delta_t);
% end

%residuoV_RE3 = R_i'*(V_j - V_i + g_vector*(delta_t*nro_intervalos_u) - V_i*delta_t*nro_intervalos_u) - (deltaV_monio(i,j,acel_lin,bias_ace,vel_ang,bias_gyr,delta_t) + deriv_deltaV_g*delta_bias_g + deriv_deltaV_a*delta_bias_a);
%residuoV_RE3 = Ri'*(V_j - V_i - g_vector*(delta_t*nro_intervalos_u)) - (deltaV_monio(i,j,acel_lin,bias_ace,vel_ang,bias_gyr,delta_t) + 0 + 0);

Rij_J=eye(3);
for s=1:actual_frame
        wi = [x(6*(s-1)+4);x(6*(s-1)+5);x(6*(s-1)+6)];
        Rij_J = Rij_J*mapeo_exponencial_SO3(wi*delta_t_ij);
end

Rij_I=eye(3);
if (actual_frame>1)
    for s=2:actual_frame
        wi = [x(6*(s-2)+4);x(6*(s-2)+5);x(6*(s-2)+6)];
        Rij_I = Rij_I*mapeo_exponencial_SO3(wi*delta_t_ij);
    end
end


V_j=Rij_J*V_j;
V_i=Rij_I*V_i;

residuoV_RE3 = (V_j - V_i - g_vector*(delta_t*nro_intervalos_u)) - (deltaV_monio(i,j,acel_lin,bias_ace,vel_ang,bias_gyr,delta_t) + 0 + 0);

res = residuoV_RE3;

end

% El resultado es un vector de 3x1
