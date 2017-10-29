function [V] = con2vert(A,b)

% Convert 2D convex polygon from half-space representation to vertex
% representation.

assert(size(A,1) == size(b,1));
assert(size(A,2) == 2);
assert(size(b,2) == 1);

V = zeros(0,2);
for C = nchoosek(1:length(b),2)'
    if abs(det(A(C,:))) > 1e-10
        x = A(C,:)\b(C);
        if all(A*x-b < 1e-5)
            V = [V; x'];
        end
    end
end

return



