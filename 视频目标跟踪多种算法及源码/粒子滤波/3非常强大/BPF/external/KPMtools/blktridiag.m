function A = blktridiag(Amd,Asub,Asup,n)
% BLKTRIDIAG: computes a sparse (block) tridiagonal matrix with n blocks
% usage: A = BLKTRIDIAG(Amd,Asub,Asup,n)
%
% arguments: (input)
%  Amd  - pxp array, forming the main diagonal blocks
%         Amd must be a square matrix
%
%  Asub - pxp array, sub diagonal block
%         Asub must be the same size and shape as Amd
%
%  Asup - pxp array, super diagonal block
%         Asup must be the same size and shape as Amd
%
%  n    - scalar integer, defines the number of blocks
%         When n == 1, only a single block will be formed, A == Amd
%
% arguments: (output)
%  A    - (n*p by n*p) SPARSE block tridiagonal array
%         If you prefer that A be full, use full(A) afterwards.
%
%
% Example 1:
%  Compute the simple 10x10 tridiagonal matrix, with 2 on the
%  diagonal, -1 on the off diagonal.
%
%  A = blktridiag(2,-1,-1,10);
%
%
% Example 2:
%  Compute the 5x5 lower bi-diagonal matrix, with blocks of
%  [1 1;1 1] on the main diagonal, [2 2;2 2] on the sub-diagonal,
%  and blocks of zeros above.
%
%  A = blktridiag(ones(2),2*ones(2),zeros(2),5);
%
%
% See also: blkdiag
%
% Author: John D'Errico
% e-mail address: woodchips@rochester.rr.com
% Release: 1.0
% Release date: 4/01/06

% get matrix sizes, check for consistency
[p,q] = size(Amd);
if (p~=q) || (p<1)
  error 'Blocks must be (non-empty) square arrays or scalars'
end
if any(p~=size(Asub)) || any(p~=size(Asup)) 
  error 'Amd, Asub, Asup are not identical in size'
end

if isempty(n) || (length(n)>1) || (n<1) || (n~=floor(n))
  error 'n must be a positive scalar integer'
end

% generate using sparse. first the main diagonal
[ind1,ind2,ind3]=ndgrid(0:p-1,0:p-1,0:n-1);
rind = 1+ind1(:)+p*ind3(:);
cind = 1+ind2(:)+p*ind3(:);
v = repmat(Amd(:),n,1);
% then the sub and super diagonal blocks.
if n>1
  % sub-diagonal
  [ind1,ind2,ind3]=ndgrid(0:p-1,0:p-1,0:n-2);
  rind = [rind;1+p+ind1(:)+p*ind3(:)];
  cind = [cind;1+ind2(:)+p*ind3(:)];
  v=[v;repmat(Asub(:),n-1,1)];
  
  % super-diagonal
  [ind1,ind2,ind3]=ndgrid(0:p-1,0:p-1,0:n-2);
  rind = [rind;1+ind1(:)+p*ind3(:)];
  cind = [cind;1+p+ind2(:)+p*ind3(:)];
  v=[v;repmat(Asup(:),n-1,1)];
end
% all in one call to sparse
A = sparse(rind,cind,v,n*p,n*p);



