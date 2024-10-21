% Author Information: 
% Hendrik Dirks
% Institute for Computational and Applied Mathematics
% University of Muenster, Germany
%
% Contact: hendrik.dirks@wwu.de
%
%
% Version 1.0
% Date: 2015-06-17

% All Rights Reserved
%
% Permission to use, copy, modify, and distribute this software and its
% documentation for any purpose other than its incorporation into a
% commercial product is hereby granted without fee, provided that the
% above copyright notice appear in all copies and that both that
% copyright notice and this permission notice appear in supporting
% documentation, and that the name of the author and University of Muenster not be used in
% advertising or publicity pertaining to distribution of the software
% without specific, written prior permission.
% 
% The gradient in dimension i can be built by applying kronecker products on the sequence
% eye(dims(n)),...,eye(dims(i+1)),Dx(dims(i)),eye(dims(i-1)),...,eye(dims(1))
function [ gradientComplete] = generateForwardGradND( dims,stepsize,varargin )
    gradientComplete = [];
    
    if (nargin > 2) %verifica el nro de argumentos que tiene la funcion
        %if third agument exists, then this specifies a specific dimension
        i = varargin{1};
        
        % Vector columna de (-1)'s y otro de (1)'s: 
        
%                                             -1 1
%                                             -1 1
%                                             -1 1
%                                               .
%                                               .
%                                               .

%                       S = spdiags(A,d,m,n) 
%   Crea una matriz esparsa"S" de mxn, tomando las comunas de A y ubicando sus valores según "d"

        gradMat = 1/stepsize(i) .* spdiags([-ones(dims(i), 1) ones(dims(i), 1)], 0:1, dims(i), dims(i));
        gradMat(end,:) = 0;

        if (i==1)
            grad = gradMat;
        else
            grad = speye(dims(1));  % Matriz identidad de "dims(1)" elementos
        end

        for j=2:numel(stepsize)
            if (j==i)
                grad = kron(gradMat,grad);
            else
                grad = kron(speye(dims(j)),grad);
            end
        end
        gradientComplete = [gradientComplete;grad];
    else
        for i=1:numel(stepsize)
            gradMat = 1/stepsize(i) .* spdiags([-ones(dims(i), 1) ones(dims(i), 1)], 0:1, dims(i), dims(i));
            gradMat(end,:) = 0;

            if (i==1)
                grad = gradMat;
            else
                grad = speye(dims(1));
            end

            for j=2:numel(stepsize)
                if (j==i)
                    grad = kron(gradMat,grad);  
          % Computa el producto de kronecker entre "gradMat" y "grad"
          % Si A es mxn y B es pxq, entonces el producto de kronecker entre ambas         
                                                  
                else
                    grad = kron(speye(dims(j)),grad);
                end
            end
            gradientComplete = [gradientComplete;grad];
        end
    end
end