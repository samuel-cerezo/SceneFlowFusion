%represents the term
%\alpha |\nabla u|_{H_\epsilon} (huber norm)
%corresponds to one primal variable u
classdef huberGradientAndRTerm < basicGradientRTerm & HuberProxDual
    properties
        epsi
    end

    methods
        function obj = huberGradientAndRTerm(alpha,dims,epsi,varargin)
            obj = obj@basicGradientRTerm(alpha,dims,varargin);
            obj.epsi = epsi;
        end

    end
end
