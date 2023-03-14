% needs the 3D positions of points in the refernce frame and the poses of 
% current frame and reference frame and gives the optical flow in return.

function [salida]= estimation_bias_gyr(X0,Y0,Z0,X1,Y1,Z1,noisy_w,delta_t)

N = 10; %numero de repeticiones

error = zeros(3,N);
salida = zeros(3,1);

for j=1:N

    % ------------------- Nube de puntos del frame 0 ---------------------
    X0 = reshape(X0,[numel(X0),1]);
    Y0 = reshape(Y0,[numel(Y0),1]);
    Z0 = reshape(Z0,[numel(Z0),1]);
    xyzPoints_0= single([X0(:),Y0(:),Z0(:)]);
        
    ptcloud0 = pointCloud(xyzPoints_0);
    %Cloud0 = pcdownsample(ptcloud0,'random',0.9);
    %Cloud0 = pcdownsample(ptcloud0,'gridAverage',2);
    Cloud0 = pcdownsample(ptcloud0,'nonuniformGridSample',6);

    % ------------------- Nube de puntos del frame 1 ---------------------
    X1 = reshape(X1,[numel(X1),1]);
    Y1 = reshape(Y1,[numel(Y1),1]);
    Z1 = reshape(Z1,[numel(Z1),1]);
    xyzPoints_1= single([X1(:),Y1(:),Z1(:)]);
        
    ptcloud1 = pointCloud(xyzPoints_1);
    %Cloud1 = pcdownsample(ptcloud1,'random',0.9);
    %Cloud1 = pcdownsample(ptcloud1,'gridAverage',2);
    Cloud1 = pcdownsample(ptcloud1,'nonuniformGridSample',6);

    %pcshow(Cloud1);

    T01 = pcregistericp(Cloud0,Cloud1,'Extrapolate',true);

    Rot_01 = T01.Rotation;

    w01_gorro = (Rot_01 -  eye(3)) / (delta_t);   %mapeo logaritmico para ir del grupo de LIE hacia el álgebra de LIE

    w01 = [-w01_gorro(2,3) ; w01_gorro(1,3) ; -w01_gorro(1,2)];

    error(1:3,j) = noisy_w - w01;

    %suma ponderada

    p1= 1 / (  abs(error(1,j)-error(2,j)) + abs(error(1,j)-error(3,j)) );
    p2 = 1 / (  abs(error(2,j)-error(1,j)) + abs(error(2,j)-error(3,j)) );
    p3 = 1 / (  abs(error(3,j)-error(2,j)) + abs(error(1,j)-error(3,j)) );

   	error(4,j) = ( p1*error(1,j) + p2*error(2,j) + p3*error(3,j) ) / (p1+p2+p3);
    
end

salida(1) = mean(error(1,:));
salida(2) = mean(error(2,:));
salida(3) = mean(error(3,:));
salida(4) = mean(error(4,:));


end