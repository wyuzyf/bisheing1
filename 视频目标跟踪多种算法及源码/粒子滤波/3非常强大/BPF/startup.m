disp('running custom startup.m')

toolboxDir = pwd;

fp{1} = toolboxDir;
fp{2} = strcat(toolboxDir, 'external'); % lightspeed, KPMtools

for i=1:length(fp)
  if(exist(fp{i}))
    addpath(genpathKO(fp{i}));
  end
end

clear fp i;

dbstop if error;

disp('done custom startup.m');
