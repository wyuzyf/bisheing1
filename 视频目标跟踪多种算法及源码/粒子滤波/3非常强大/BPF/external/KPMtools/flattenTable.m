function [ndx, value] = flattenTable(T)

sz = size(T);
p = length(sz);
S = prod(sz);
ndx = zeros(S, p);
value   = zeros(1,S);
for i=1:S
  ndx(i,:) = ind2subv(sz, i);
  value(i) = T(i);
end
