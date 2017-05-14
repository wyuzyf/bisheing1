function bar_posu = predictstates(bar_vel,bar_pos,deta);
% PURPOSE : Performs the prediction step of the sequential SIR algorithm for 
%         : the model described in the file sirdemo1.m.
% INPUTS  : - x = The state samples.
%           - t = The current time step.
%           - Q = The variance of the process noise.
% OUTPUTS : - xu = The state samples after the prediction step.

% AUTHOR  : Nando de Freitas - Thanks for the acknowledgement :-)
% DATE    : 08-09-98

if nargin < 2, error('Not enough input arguments.'); end

% w = sqrt(Q)*randn(size(x));
% xu = 0.5.*x + 25.*x./(1+x.^(2)) + 8*cos(1.2*(t)).*ones(size(x)) + w;

bar_posu=mod((bar_pos+deta.*bar_vel),1);
