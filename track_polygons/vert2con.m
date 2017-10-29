function [A,b] = vert2con(V)

% Convert 2D convex polygon from vertex representation to half-space
% representation.

assert(size(V,2) == 2);

K = convhull(V);
A = nan(length(K)-1,2);
b = nan(length(K)-1,1);
for i = 1:(length(K)-1)
    n = [0 1;-1 0]*(V(K(i+1),:) - V(K(i),:))';
    x0 = V(K(i),:);
    b(i) = x0*n;
    A(i,:) = n';
end

% normalize
L = sqrt(sum(A.^2,2));
A = A ./ repmat(L,1,size(A,2));
b = b ./ L;
return

