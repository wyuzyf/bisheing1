function dispTable(T)

sz = size(T);
for i=1:prod(sz)
  ndx = ind2subv(sz, i);
  fprintf(1, '%d ', ndx);
  fprintf(1, ':%6.4f\n', T(i));
end
