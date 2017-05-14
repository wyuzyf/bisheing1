function boxplotKPM(data, varargin)
% boxplotKPM  Like the built-in function but works even if there is only one datapoint (row) per method

[Ntrials Nmethods] = size(data);
if Ntrials == 1
  data = [data; data];
end
boxplot(data, varargin{:});
