% needs the 3D positions of points in the refernce frame and the poses of 
% current frame and reference frame and gives the optical flow in return.

function w01 = vel_ang_ICP(X0,Y0,Z0,X1,Y1,Z1,delta_t)

% ------------------- Nube de puntos del frame 0 ---------------------
xyzPoints_0 = single(zeros(numel(X0),3));

i=1;
for c=1:size(X0,2)
    for r=1:size(X0,1)
        xyzPoints_0(i,:) = [X0(r,c) Y0(r,c) Z0(r,c)];
        i=i+1;
    end
end
clear i;
Cloud0 = pointCloud(xyzPoints_0);

% ------------------- Nube de puntos del frame 1 ---------------------
xyzPoints_1 = single(zeros(numel(X1),3));

i=1;
for c=1:size(X1,2)
    for r=1:size(X1,1)
        xyzPoints_1(i,:) = [X1(r,c) Y1(r,c) Z1(r,c)];
        i=i+1;
    end
end
clear i;
Cloud1 = pointCloud(xyzPoints_1);
%pcshow(Cloud1);

T10 = pcregistericp(Cloud1,Cloud0,'Extrapolate',true);
T01 = invert(T10);

Rot_01 = T01.Rotation;

w01_gorro = (Rot_01 -  eye(3)) / (delta_t);   %mapeo logaritmico para ir del grupo de LIE hacia el álgebra de LIE

w01 = [-w01_gorro(2,3) ; w01_gorro(1,3) ; -w01_gorro(1,2)];


end