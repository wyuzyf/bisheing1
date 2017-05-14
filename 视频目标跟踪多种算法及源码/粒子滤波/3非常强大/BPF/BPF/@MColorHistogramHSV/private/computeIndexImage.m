function [hsImage, vImage] = computeIndexImage(hsvImage, hbins, sbins, vbins)

edges = linspace(0, 1, hbins + 1);
edges(end) = inf;
[n, hbinNum] = histc(hsvImage(:, :, 1), edges);

edges = linspace(0, 1, sbins + 1);
edges(end) = inf;
[n, sbinNum] = histc(hsvImage(:, :, 2), edges);

edges = linspace(0, 1, vbins + 1);
edges(end) = inf;
[n, vImage] = histc(hsvImage(:, :, 3), edges);

hsImage = hbinNum .* (sbins-1) + sbinNum;
