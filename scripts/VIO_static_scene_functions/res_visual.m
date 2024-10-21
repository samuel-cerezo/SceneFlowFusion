function res = res_visual(x,dZ,X,Y,Z,f,NoF,i)
% Local Coordinates


[alto,ancho] = size(dZ{1,1});
%   NoF = number of frames (2...3).
%
% OBS: Haciendo coincidir las coord del mundo con las de la cámara "i",
% entonces...

%   bias_gyr = [x(end-5), x(end-4), x(end-3)]';
% vx = x(end-((NoF-i+1)*6)+8-1);

v = [x(6*(i-1)+1);x(6*(i-1)+2);x(6*(i-1)+3)]; %The first 9 places in the state x are for R matrix.
w = [x(6*(i-1)+4);x(6*(i-1)+5);x(6*(i-1)+6)];

% Rij = reshape(x(1:9),[3,3]);
% if (i>1)
%     v = Rij*v;
%     w = Rij*w;
% end

vx = v(1);    
vy = v(2);
vz = v(3);
wx = w(1);
wy = w(2);
wz = w(3);

%res = (1 + dZ{1,i}.*((X{1,i}*f{1,i})./(Z{1,i}.^2)) + dZ{2,i}.*((Y{1,i}*f{2,i})./(Z{1,i}.^2))).*(x(6*(i-1)+3+9) + Y{1,i}*x(6*(i-1)+4+9) - X{1,i}*x(6*(i-1)+5+9)) + (f{1,i})*(dZ{1,i}./Z{1,i}).*(-x(6*(i-1)+1+9) + Y{1,i}*x(6*(i-1)+6+9) - Z{1,i}*x(6*(i-1)+5+9)) + (f{2,i})*(dZ{2,i}./Z{1,i}).*(-x(6*(i-1)+2+9) - X{1,i}*x(6*(i-1)+6+9) + Z{1,i}*x(6*(i-1)+4+9)) + dZ{3,i};
%res = (1 + dZ{1,i}.*((X{1,i}*f{1,i})./(Z{1,i}.^2)) + dZ{2,i}.*((Y{1,i}*f{2,i})./(Z{1,i}.^2))).*(x(6*(i-1)+3+9) + Y{1,i}*x(6*(i-1)+4+9) - X{1,i}*x(6*(i-1)+5+9)) + (f{1,i})*(dZ{1,i}./Z{1,i}).*(-x(6*(i-1)+1+9) + Y{1,i}*x(6*(i-1)+6+9) - Z{1,i}*x(6*(i-1)+5+9)) + (f{2,i})*(dZ{2,i}./Z{1,i}).*(-x(6*(i-1)+2+9) - X{1,i}*x(6*(i-1)+6+9) + Z{1,i}*x(6*(i-1)+4+9)) + dZ{3,i};
res = (1 + dZ{1,i}.*((X{1,i}*f{1,i})./(Z{1,i}.^2)) + dZ{2,i}.*((Y{1,i}*f{2,i})./(Z{1,i}.^2))).*(vz + Y{1,i}*wx - X{1,i}*wy) + (f{1,i})*(dZ{1,i}./Z{1,i}).*(-vx + Y{1,i}*wz - Z{1,i}*wy) + (f{2,i})*(dZ{2,i}./Z{1,i}).*(-vy - X{1,i}*wz + Z{1,i}*wx) + dZ{3,i};

res = reshape(res,[ancho*alto 1]);

end
