function obj = setnbins(obj, nbins)
% SETNBINS      Set the number of bins
%   obj = setnbins(obj, nbins) set the number of bins of 'obj'.
%

% Copyright 2006 Wei-Lwun Lu
% setnbins.m version 1.0

if nbins >= max(max(obj.indexImage)) || isempty(obj.indexImage)
    obj.nbins = nbins;
else
    error('Invalid number of bins.');
end
    
