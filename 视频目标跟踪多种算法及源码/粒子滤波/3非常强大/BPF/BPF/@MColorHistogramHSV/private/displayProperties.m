function displayProperties(strlist)

fprintf('Properties:\n');
for k = 1 : length(strlist)
    fprintf('\t %s\n', strlist{k});
end
