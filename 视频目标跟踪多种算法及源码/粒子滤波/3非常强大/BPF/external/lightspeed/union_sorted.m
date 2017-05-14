function [c,a_match,b_match] = union_sorted(a,b)
%UNION_SORTED  Set union of sorted sets.
% UNION_SORTED(A,B) where A and B are vectors returns the combined values
% from A and B with no repetitions.  A (and B) must be sorted and unique, and 
% the result will be sorted and unique.
%
% [C,A_MATCH,B_MATCH] = UNION_SORTED(A,B) also returns
%   A_MATCH = MATCH_SORTED(A,C)
%   B_MATCH = MATCH_SORTED(B,C)
%
% Examples:
%   union_sorted([10 20 30], [20 30 40])
%   [c,a_match,b_match] = union_sorted([10 20 30], [20 30 40])

% Written by Tom Minka
% (c) Microsoft Corporation. All rights reserved.

% instead of a full sort, you could do a merge of the two sorted lists.
if nargout <= 1
  c = sort([a(~ismember_sorted(a,b)) b]);
else
  [tf,loc] = ismember_sorted(a,b);
  [c,i] = sort([a(~tf) b]);
  nc = length(c);
  nb = length(b);
  na = length(a);
  b_match = i((nc-nb+1):nc);
  a_match = zeros(1,na);
  a_match(~tf) = i(1:(nc-nb));
  a_match(tf) = b_match(loc(tf));
end
