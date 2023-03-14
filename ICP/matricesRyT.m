

clear;
delta_t = 1/30;

carpeta_rgb = 'Dataset_ICL_NUIM/traj1_frei_png/rgb/';
carpeta_profundidad = 'Dataset_ICL_NUIM/traj1_frei_png/depth/';
carpeta = 'Dataset_ICL_NUIM/office_room_traj1_loop/';



[R_1,T_1] = matrices_RT(strcat(carpeta,'scene_318','.txt')); 	
[R0,T0] = matrices_RT(strcat(carpeta,'scene_319','.txt')); 	
[R1,T1] = matrices_RT(strcat(carpeta,'scene_320','.txt'));

[X_1,Y_1,Z_1] = compute3Dpositions(strcat(carpeta,'scene_318','.txt'),strcat(carpeta,'scene_318','.depth'));
X_1 = reshape(X_1,[numel(X_1),1]);
Y_1 = reshape(Y_1,[numel(Y_1),1]);
Z_1 = reshape(Z_1,[numel(Z_1),1]);
xyzPoints_1= single([X_1(:),Y_1(:),Z_1(:)]);
Cloud_1 = pointCloud(xyzPoints_1);
% Cloud_1 = pcdenoise(Cloud_1);
% Cloud_1 = pcdownsample(Cloud_1,'nonuniformGridSample',6);



[X0,Y0,Z0] = compute3Dpositions(strcat(carpeta,'scene_319','.txt'),strcat(carpeta,'scene_319','.depth'));
X0 = reshape(X0,[numel(X0),1]);
Y0 = reshape(Y0,[numel(Y0),1]);
Z0 = reshape(Z0,[numel(Z0),1]);
xyzPoints0= single([X0(:),Y0(:),Z0(:)]);
Cloud0 = pointCloud(xyzPoints0);
% Cloud0 = pcdenoise(Cloud0);
% Cloud0 = pcdownsample(Cloud0,'nonuniformGridSample',6);


[X1,Y1,Z1] = compute3Dpositions(strcat(carpeta,'scene_320','.txt'),strcat(carpeta,'scene_320','.depth'));
X1 = reshape(X1,[numel(X1),1]);
Y1 = reshape(Y1,[numel(Y1),1]);

Z1 = reshape(Z1,[numel(Z1),1]);
xyzPoints1= single([X1(:),Y1(:),Z1(:)]);
Cloud1 = pointCloud(xyzPoints1);
% Cloud1 = pcdenoise(Cloud1);
% 
% Cloud1 = pcdownsample(Cloud1,'nonuniformGridSample',6);



T_10 = pcregistericp(Cloud_1,Cloud0);
T01 = pcregistericp(Cloud0,Cloud1);
T_11 = pcregistericp(Cloud_1,Cloud1);


real = T0-T_1
estimacion = T_10.Translation'


% %Rot_01 = T01.Rotation;
% tras_01 = T01.Translation;
% tras_12 = T12.Translation;
% 
% v1 = tras_01' / delta_t;
% %v1 = (T1-T0)/delta_t;
% 
% v2 = tras_12' / delta_t;
% %v2 = (T2-T1)/delta_t;
% aceleracion = ((v2 - v1 ) / (delta_t));
