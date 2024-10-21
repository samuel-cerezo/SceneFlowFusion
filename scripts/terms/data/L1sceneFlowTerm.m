%term representing alpha |u_t + \nabla u\cdot v|_1, where v is the unknown
classdef L1sceneFlowTerm < basicSceneFlow & L1DataProxDual

    methods
        function obj = L1sceneFlowTerm(alpha,image1,image2,varargin)
            obj = obj@basicSceneFlow(alpha,image1,image2,varargin);
        end
    end
end