function ndx = findrows(M, v)
% Find the rows of M that are equal to v

[nr nc] = size(M);
diff = M - repmat(v(:)', nr, 1);
A = sum(diff, 2);
ndx = find(A==0);
