
% We choose a quadratic expression that penalizes the second 
%  derivatives homogeneously for all pixels.

% k_l = empiric constant


function [result] = mask_penalize(id0,delta_t,k_l) 

    %k_l = 100000;
    [Zuu,Zvv,Zuv,Ztu,Ztv] = second_partial_derivates(id0,delta_t);
    result = k_l*(delta_t^2*(Ztu.^2 + Ztv.^2) + Zuu.^2 + Zvv.^2 + Zuv.^2); 

end
