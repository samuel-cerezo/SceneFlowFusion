%represents the term
%\alpha |\nabla u|_{H_\epsilon} (huber norm)
%corresponds to one primal variable u
classdef huberGradientAndRTerm_con_Operador < basicGradientRTerm_con_Operador & HuberProxDual
    properties
        epsi
    end

    methods
        function obj = huberGradientAndRTerm_con_Operador(alpha,dims,epsi,operadorX, operadorY,varargin)
            obj = obj@basicGradientRTerm_con_Operador(alpha,dims,operadorX, operadorY,varargin);
            obj.epsi = epsi;
        end

    end
end
