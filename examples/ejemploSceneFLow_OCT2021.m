%% Datos

addpath(genpath('..'));
clear id0
clear id1
%-------------------------   Dataset OXFORD
%-----------------------------
% %%%%%% Con movimiento de la camara
% carpeta = 'Dataset_OXFORD/data/occlusion_2_translational/rgbd/';
% id0 = 417;

% id1 = 420;

% %%%%%% Sin movimiento de la camara
% carpeta = 'Dataset_OXFORD/data/occlusion_2_static/rgbd/';
% id0 = 273;
% id1 = 277;

% Leo imagenes de camara

% id0 = string(id0);
% id1 = string(id1);close

% i0 = imread(strcat(carpeta,'000',id0,'_color.png'));
% i1 = imread(strcat(carpeta,'000',id1,'_color.png'));
% % Leo imagenes de profundidad
% z0 = imread(strcat(carpeta,'000',id0,'_aligned_depth.png'));
% z1 = imread(strcat(carpeta,'000',id1,'_aligned_depth.png'));
%--------------------------------------------------------------------------

%----------------------   Dataset ICL-NUIM   ------------------------------
id0 = 319;
id1 = 320;


carpeta_rgb = 'Dataset_ICL_NUIM/traj1_frei_png/rgb/';
carpeta_profundidad = 'Dataset_ICL_NUIM/traj1_frei_png/depth/';
% Leo imagenes de camara
id0 = string(id0);
id1 = string(id1);
i0 = imread(strcat(carpeta_rgb,id0,'.png'));
i1 = imread(strcat(carpeta_rgb,id1,'.png'));
% Leo imagenes de profundidad
z0 = imread(strcat(carpeta_profundidad,id0,'.png'));
z1 = imread(strcat(carpeta_profundidad,id1,'.png'));
%--------------------------------------------------------------------------

if (size(i0,3) > 1)
    i0 = rgb2gray(i0); 
end
if (size(i1,3) > 1)
    i1 = rgb2gray(i1);
end
if (size(z0,3) > 1)
    z0 = rgb2gray(z0);
end
if (size(z1,3) > 1)
    z1 = rgb2gray(z1);
end

i0 = im2double(i0);
i1 = im2double(i1);
z0 = im2double(z0);
z1 = im2double(z1);

% i0 = i0*(max(max(z0)))/(max(max(i0)));

dim = size(i0);
alto = dim(1);
ancho = dim(2);

%f1 = imresize(f1,2);
%f2 = imresize(f2,2);

figure();imagesc(i0);axis image;colormap(gray);title('Imagen 0')
figure();imagesc(z0);axis image;colormap(gray);title('Profundidad 0')
figure();imagesc(i1);axis image;colormap(gray);title('Imagen 1')
figure();imagesc(z1);axis image;colormap(gray);title('Profundidad 1')

%% Algoritmo piramidal con rx y ry 
%---------------------------------------------------------------------------------------------------------------------------------------------
%---------------------------------------------------------------------------------------------------------------------------------------------
%---------------------------------------------------------------------------------------------------------------------------------------------

i0=imresize(i0,0.25,'method','bicubic');
z0=imresize(z0,0.25,'method','bicubic');
i1=imresize(i1,0.25,'method','bicubic');
z1=imresize(z1,0.25,'method','bicubic');
% Reduje las dimensiones a 120x160

dim = size(i0); % [m n]
alto = dim(1);
ancho = dim(2);
nro_elementos = prod(dim);  %Cantidad de elementos en cada imagen (N)

lambda_I = 0.4;    % Constantes para los terminos regularizadores (0.04)
lambda_D = 0.35;   %(0.35)

%-----------------  Inicializamos las variables ---------------------------
pyramidSceneFlow ={};       %Contiene las variables primas obtenidas en cada iteracion
pyramidDualSeed = {};       %Contiene las variables duales obtenidas en cada iteracion
pyramidImages = {};         %Contiene las imagenes sub-sampleadas
nro_elementos = prod(dim);  %Cantidad de elementos en cada imagen

%-------------------- Estructura de la piramide ---------------------------
nro_iter = 4;               %Cantidad de niveles de la piramide
factor_reduccion=0.5;       %Reduccion entre iteraciones
factor_ampliacion=1/factor_reduccion;
%metodo='nearest';  %es el que peor resultado da
%metodo='bilinear';
metodo='bicubic';


pyramidImages{1,nro_iter}=i0;   %Inicializamos base de la piramide
pyramidImages{2,nro_iter}=i1;   %Inicializamos base de la piramide
pyramidImages{3,nro_iter}=z0;   %Inicializamos base de la piramide
pyramidImages{4,nro_iter}=z1;   %Inicializamos base de la piramide

for i=1:(nro_iter-1)
    pyramidImages{1,nro_iter-i} = imresize(pyramidImages{1,nro_iter-i+1},factor_reduccion,'method',metodo);
    pyramidImages{2,nro_iter-i} = imresize(pyramidImages{2,nro_iter-i+1},factor_reduccion,'method',metodo);
    pyramidImages{3,nro_iter-i} = imresize(pyramidImages{3,nro_iter-i+1},factor_reduccion,'method',metodo);
    pyramidImages{4,nro_iter-i} = imresize(pyramidImages{4,nro_iter-i+1},factor_reduccion,'method',metodo);    
end
%--------------------------------------------------------------------------


for pyramid_iter=1:nro_iter
    
    clear v1, clear v2, clear v3;
    
    dim = size(pyramidImages{1,pyramid_iter});
    alto = dim(1);
    ancho = dim(2);
    nro_elementos = prod(dim);  %Cantidad de elementos en cada imagen

    if (pyramid_iter==1)
         inicial_primal{1} = zeros(nro_elementos,1);
         inicial_primal{2} = zeros(nro_elementos,1);
         inicial_primal{3} = zeros(nro_elementos,1);
%         inicial_dual{1} = zeros(nro_elementos,1);
%         inicial_dual{2} = zeros(nro_elementos,1);
%         inicial_dual{3} = zeros(nro_elementos,1);
    else
        %----- Variables primas y duales obtenidas en la iteración anterior  ------        
        inicial_primal{1} = reshape(imresize(pyramidSceneFlow{1,pyramid_iter - 1},factor_ampliacion,'method',metodo),[nro_elementos,1]);%i0_agrandada = imresize(pyramidSceneFlow{1,pyramid_iter - 1},factor_ampliacion);
        inicial_primal{2} = reshape(imresize(pyramidSceneFlow{2,pyramid_iter - 1},factor_ampliacion,'method',metodo),[nro_elementos,1]);
        inicial_primal{3} = reshape(imresize(pyramidSceneFlow{3,pyramid_iter - 1},factor_ampliacion,'method',metodo),[nro_elementos,1]);
%         inicial_dual{1} = reshape(imresize(pyramidDualSeed{1,pyramid_iter - 1},factor_ampliacion,'method',metodo),[nro_elementos,1]);
%         inicial_dual{2} = reshape(imresize(pyramidDualSeed{2,pyramid_iter - 1},factor_ampliacion,'method',metodo),[nro_elementos,1]);
%         inicial_dual{3} = reshape(imresize(pyramidDualSeed{3,pyramid_iter - 1},factor_ampliacion,'method',metodo),[nro_elementos,1]);

    end

    
    % pyramidImages{3,pyramid_iter}=z0;
    
    
    
    carpeta = 'Dataset_ICL_NUIM/office_room_traj1_loop/';
    K = matriz_K(strcat(carpeta,'scene_',id0,'.txt'),ancho,alto);   %Matriz intrínseca de la cámara

    fx = K(1,1);    % Parametros de la cámara
    fy = K(2,2);   
    u0 = K(1,3);
    v0 = K(2,3);

    [dZdx_m, dZdx_M, dZdy_m, dZdy_M] = derivadas_laterales(pyramidImages{3,pyramid_iter});
    [rx_m, rx_M, ry_m, ry_M] = coef_R_laterales(pyramidImages{3,pyramid_iter},K);

    dZdx =  ((rx_M.*dZdx_M + rx_m.*dZdx_m))./(rx_m+rx_M);
    dZdy = ((ry_M.*dZdy_M + ry_m.*dZdy_m))./(ry_m+ry_M);

    % [dZdx,dZdy] = imgradientxy(z0);  %gradiente de z0

    aux_u = repmat([1:ancho],alto,1);
    aux_v = repmat([1:alto]',1,ancho);

    % u_u0_by_fx = (aux_u - u0)/fx;   %expresado en unidades métricas
    % v_v0_by_fy = (aux_v - v0)/fy;

    x = double(aux_u) - u0*ones(alto,ancho);
    y = double(aux_v) - v0*ones(alto,ancho);

    dXdx = (1/fx)*(pyramidImages{3,pyramid_iter} + x.*dZdx);
    dYdy = (1/fy)*(pyramidImages{3,pyramid_iter} + y.*dZdy);

    %   Coeficientes que se multiplican por los gradientes de u,v y w.
    rx = 1./sqrt(dXdx.^2 + dZdx.^2);
    rx = rx/(max(max(rx)));
    ry = 1./sqrt(dYdy.^2 + dZdy.^2);
    ry = ry/(max(max(ry)));
    
    rx = rx';
    rx = reshape(rx,[1 nro_elementos]);
    rx = repmat(rx,nro_elementos,1);
    ry = ry';
    ry = reshape(ry,[1 nro_elementos]);
    ry = repmat(ry,nro_elementos,1);


    % --------- Aplicamos derivadas considerando profundidad ---------
    operador{1} = sparse(matriz_derivada_X(pyramidImages{3,pyramid_iter},K));   % NxN
    operador{2} = sparse(matriz_derivada_Y(pyramidImages{3,pyramid_iter},K));   % NxN
    operador{1} = operador{1}.*rx;  % .*rx;
    operador{2} = operador{2}.*ry;  % .*ry

 
    
    %----------------------------------------------------------------------
    main = flexBox; %inicializa el objeto de Flexbox

    % Para ejecución en C++...
    main.params.tryCPP = 0; %change, if C++ module is compiled
    main.params.verbose = 1; %change, if C++ module is compiled

    %Agregamos las variables primas: 
    %     v1 <--> u
    %     v2 <--> v
    %     v3 <--> w
    v1 = main.addPrimalVar(dim);  
    v2 = main.addPrimalVar(dim);
    v3 = main.addPrimalVar(dim);
    %----------------------------------------------------------------------
    
    main.seedPrimal(inicial_primal);    %Carga los valores iniciales de las variables primas

    %---------------- Agrego los términos de consistencia  ----------------
    b1=main.addTerm(L1opticalFlowTerm(1,pyramidImages{1,pyramid_iter},pyramidImages{2,pyramid_iter}),[v1,v2]);   %Consistencia en brillo
    b2=main.addTerm(L1sceneFlowTerm(1,pyramidImages{3,pyramid_iter},pyramidImages{4,pyramid_iter}),[v1,v2,v3]);  %Consistencia en forma
    
    nro_varDuales = b1+b2;
    
%-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------   
% -------------------------------------------------------------- Agrego los operadores que incluyen derivadas laterales y las r's ----------------------------------------------------------------------------------------    
%     id0 = 210; id0=string(id0);
%     id1 = 213; id1=string(id1);
%     carpeta = 'Dataset_ICL_NUIM/living_room_traj3_loop/';
%     K = matriz_K(strcat(carpeta,'scene_00_0',id0,'.txt'),ancho,alto);   %Matriz intrínseca de la cámara
% 
%     fx = K(1,1);    % Parametros de la cámara
%     fy = K(2,2);   
%     u0 = K(1,3);
%     v0 = K(2,3);
% 
%     [dZdx_m, dZdx_M, dZdy_m, dZdy_M] = derivadas_laterales(pyramidImages{3,pyramid_iter});
%     [rx_m, rx_M, ry_m, ry_M] = coef_R_laterales(pyramidImages{3,pyramid_iter},K);
% 
%     dZdx =  ((rx_M.*dZdx_M + rx_m.*dZdx_m))./(rx_m+rx_M);
%     dZdy = ((ry_M.*dZdy_M + ry_m.*dZdy_m))./(ry_m+ry_M);
% 
% 
%     aux_u = repmat([1:ancho],alto,1);
%     aux_v = repmat([1:alto]',1,ancho);
% 
%     % u_u0_by_fx = (aux_u - u0)/fx;   %expresado en unidades métricas
%     % v_v0_by_fy = (aux_v - v0)/fy;
% 
%     x = double(aux_u) - u0*ones(alto,ancho);
%     y = double(aux_v) - v0*ones(alto,ancho);
% 
%     dXdx = (1/fx)*(pyramidImages{3,pyramid_iter} + x.*dZdx);
%     dYdy = (1/fy)*(pyramidImages{3,pyramid_iter}+ y.*dZdy);
% 
%     %   Coeficientes que se multiplican por los gradientes de u,v y w.
%     rx = 1./sqrt(dXdx.^2 + dZdx.^2);
%     rx = rx/(max(max(rx)));           
%     ry = 1./sqrt(dYdy.^2 + dZdy.^2);
%     ry = ry/(max(max(ry)));    
%        
%     rx = reshape(rx,[nro_elementos 1]);
%     ry = reshape(ry,[nro_elementos 1]);
%     rx = reshape(rx,[1 nro_elementos]);
%     rx = repmat(rx,nro_elementos,1);
%     ry = reshape(ry,[1 nro_elementos]);
%     ry = repmat(ry,nro_elementos,1);

    % --------- Aplicamos derivadas considerando profundidad ---------
%     operador{1} = sparse(matriz_derivada_X(pyramidImages{3,pyramid_iter},K));   % NxN
%     operador{2} = sparse(matriz_derivada_Y(pyramidImages{3,pyramid_iter},K));   % NxN
%     operador{1} = operador{1}.*rx;
%     operador{2} = operador{2}.*ry;   
    %---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    
    %------------ Agrego los regularizadores a cada variable --------------
%     main.addTerm(huberGradient(operador{1}, operador{2}, lambda_I,dim,0.01),v1);     %alfa=.04 (jaimez)
%     main.addTerm(huberGradient(operador{1}, operador{2}, lambda_I,dim,0.01),v2);     %alfa=.04 (jaimez)
%     main.addTerm(huberGradient(operador{1}, operador{2}, lambda_D,dim,0.01),v3);     %alfa=.35 (jaimez)
    
     main.addTerm(huberGradient(lambda_I,dim,0.01),v1);     %alfa=.04 (jaimez)
     main.addTerm(huberGradient(lambda_I,dim,0.01),v2);     %alfa=.04 (jaimez)
     main.addTerm(huberGradient(lambda_D,dim,0.01),v3);     %alfa=.35 (jaimez)
    
% main.addTerm(huberGradientAndRTerm_con_Operador(lambda_I,size(i0),0.01,operador{1},operador{2}),v1);     %alfa=.04 (jaimez)
% main.addTerm(huberGradientAndRTerm_con_Operador(lambda_I,size(i0),0.01,operador{1},operador{2}),v2);     %alfa=.04 (jaimez)
% main.addTerm(huberGradientAndRTerm_con_Operador(lambda_D,size(i0),0.01,operador{1},operador{2}),v3);     %alfa=.35 (jaimez)

%     
    
    
    
    %----------------------------------------------------------------------
      
    for j=1:nro_varDuales
        if (pyramid_iter > 1)
            inicial_dual{j} = reshape(imresize(pyramidDualSeed{j,pyramid_iter - 1},factor_ampliacion,'method',metodo),[nro_elementos,1]);
        else
            inicial_dual{j} = zeros(nro_elementos,1);
        end
    end
    
    main.seedDual(inicial_dual,nro_varDuales);    %Carga los valores iniciales de las variables duales

    %--------------- Corre el algoritmo de minimizacion -------------------
    tic;
    main.runAlgorithm();
    toc;
    %----------------------------------------------------------------------
    
    %--------------------- Obtengo el scene flow --------------------------
    pyramidSceneFlow{2,pyramid_iter} = -main.getPrimal(v1); % Guardo V      % Cambia las componentes porque MATLAB alterna los ejes  
    pyramidSceneFlow{1,pyramid_iter} = main.getPrimal(v2); % Guardo U
    pyramidSceneFlow{3,pyramid_iter} = main.getPrimal(v3); % Guardo W
    %----------------------------------------------------------------------
    
    % ---------------- Realizamos el filtrado de mediana -------------------------
    
    pyramidSceneFlow{1,pyramid_iter}  = medfilt2(pyramidSceneFlow{1,pyramid_iter} );
    pyramidSceneFlow{2,pyramid_iter}  = medfilt2(pyramidSceneFlow{2,pyramid_iter} );
    pyramidSceneFlow{3,pyramid_iter}  = medfilt2(pyramidSceneFlow{3,pyramid_iter} );

    % -------------------------------------------------------------------------------------------
    %------------- Obtenemos las variables primas y duales  -------------------
    pyramidDualSeed{1,pyramid_iter} = reshape(main.getDual(v1),dim);    %Guardo Dual1
    pyramidDualSeed{2,pyramid_iter} = reshape(main.getDual(v2),dim);    %Guardo Dual2
    pyramidDualSeed{3,pyramid_iter} = reshape(main.getDual(v3),dim);    %Guardo Dual3
end

%------------------------ Visualizo resultados ----------------------------
%scene_flow = cat(3,u,v,w);

u = pyramidSceneFlow{1,pyramid_iter};
v = pyramidSceneFlow{2,pyramid_iter};
w = pyramidSceneFlow{3,pyramid_iter};

figure();imagesc(pyramidSceneFlow{1,pyramid_iter});axis image;title(strcat('Variable U con  ',string(nro_iter),' niveles')); axis equal
figure();imagesc(pyramidSceneFlow{2,pyramid_iter});axis image;title(strcat('Variable V con  ',string(nro_iter),' niveles')); axis equal
figure();imagesc(pyramidSceneFlow{3,pyramid_iter});axis image;title(strcat('Variable W con  ',string(nro_iter),' niveles')); axis equal

[x,y] = meshgrid(1:1:ancho,1:1:alto);
y = flip(y);
figure(); quiver(x,y,pyramidSceneFlow{1,pyramid_iter},pyramidSceneFlow{2,pyramid_iter},'Color','r','LineWidth',2,'AutoScale','on','AutoScaleFactor',1); title(strcat('Campo de vectores con piramide de ',string(nro_iter),' niveles'));
%figure(3);clf;imagesc(flowToColorV2(cat(3,resultV1,resultV2),5));title('Color-coded flow field')
%% Muestra superposición entre i0 e i1

fusion_imagenes = imfuse(i0,i1,'falsecolor','Scaling','joint','ColorChannels',[1 2 0]); imshow(fusion_imagenes)

%% Obtenemos los valores de consistencia en brillo

i1_estrella = zeros(alto + 20,ancho+20);
i11 = zeros(alto + 20,ancho + 20);
i00 = zeros(alto + 20,ancho + 20);

for j=1:ancho
    for i=1:alto
        
        i11(i+10,j+10) = i1(i,j);
        i00(i+10,j+10) = i0(i,j);
    end
end


u_entero = round(u);
v_entero = round(v);

for j=1:ancho
    for i=1:alto
        
        i1_estrella(i+10,j+10) = i11(10 + i + u_entero(i,j), 10 + j + v_entero(i,j));
        
    end
end

consistencia_brillo = i00 - i1_estrella;

figure();imagesc(i00);axis image;title('i0'); axis equal
figure();imagesc(i11);axis image;title('i1'); axis equal
figure();imagesc(i1_estrella);axis image;title('i1(x+u,y+v)'); axis equal
figure();imagesc(consistencia_brillo);axis image;title('Consistencia en brillo'); axis equal

%% Obtenemos los valores de consistencia en forma

z1_estrella = zeros(alto + 20,ancho+20);
z11 = zeros(alto + 20,ancho + 20);
z00 = zeros(alto + 20,ancho + 20);
ww = zeros (alto + 20,ancho + 20);
for j=1:ancho
    for i=1:alto
        
        z11(i+10,j+10) = z1(i,j);
        z00(i+10,j+10) = z0(i,j);
        ww(i+10,j+10) = w(i,j);
    end
end


u_entero = round(u);
v_entero = round(v);

for j=1:ancho
    for i=1:alto
        
        z1_estrella(i+10,j+10) = z11(10 + i + u_entero(i,j), 10 + j + v_entero(i,j));
        
    end
end

consistencia_forma = z00 - z1_estrella + ww;

figure();imagesc(z00);axis image;title('z0'); axis equal
figure();imagesc(z1_estrella);axis image;title('z1(x+u,y+v)'); axis equal
figure();imagesc(ww);axis image;title('w'); axis equal

figure();imagesc(consistencia_forma);axis image;title('Consistencia en forma'); axis equal

%% Obtencion del Ground Truth
%----------------------   Dataset ICL-NUIM   ------------------------------

carpeta = 'Dataset_ICL_NUIM/office_room_traj1_loop/';


% ancho=640;    %640
% alto=480;       %480

ancho=160;    %640
alto=120;       %480

K0 = matriz_K(strcat(carpeta,'scene_',id0,'.txt'),alto,ancho); %Matriz intrínseca de la cámara
K1 = matriz_K(strcat(carpeta,'scene_',id1,'.txt'),alto,ancho);

[R0,T0] = matrices_RT(strcat(carpeta,'scene_',id0,'.txt')); %Matriz extrinseca de la camara
[R1,T1] = matrices_RT(strcat(carpeta,'scene_',id1,'.txt'));

[X0,Y0,Z0] = posicion_3D(strcat(carpeta,'scene_',id0,'.txt'),strcat(carpeta,'scene_',id0,'.depth'),alto,ancho);  % Cada coordenada X,Y,Z es de 480x640 c/u
[X1,Y1,Z1] = posicion_3D(strcat(carpeta,'scene_',id1,'.txt'),strcat(carpeta,'scene_',id1,'.depth'),alto,ancho);

[u_gt, v_gt] = opticalflow(X1,Y1,Z1,K0,R0,T0,R1,T1);
[x,y] = meshgrid(1:1:ancho,1:1:alto);

y = flip(y);
figure(); quiver(x,y,u_gt,v_gt,'Color','b','LineWidth',2,'AutoScale','on','AutoScaleFactor',1);title('GT');


%% Implementacion de métricas


error_u = (u - u_gt);
error_v = (v - v_gt);

error_u_2 = error_u.^2;
error_v_2 = error_v.^2;

error_u_rel = abs((error_u)./u_gt);
error_v_rel = abs((error_v)./v_gt);
%% Grafica de metricas

% ------------------------------------  Grafico Histogramas ---------------------------

% histograma de error absoluto
figure();
h_error_u_abs=histogram(error_u)
title('histograma error U ');
figure();
h_error_v_abs=histogram(error_v)
title('histograma error V ');


% histograma de error cuadratico
figure();
h_error_u_2 = histogram(error_u_2,10000)
title('histograma error U^2 ');
figure();
h_error_v_2=histogram(error_v_2,10000)
title('histograma error V^2 ');

% -------------------------------------------- Grafico errores   -------------------------
figure(), imagesc(abs(error_u));axis image;title('error en U'); axis equal;

figure(), imagesc(abs(error_v));axis image;title('error en v'); axis equal;





abs_error_u = abs(error_u);
mean(abs_error_u(:))
abs_error_v = abs(error_v);
mean(abs_error_v(:))

abs_u_gt= abs(u_gt);
mean(abs_u_gt(:))
abs_v_gt= abs(v_gt);
mean(abs_v_gt(:))




















