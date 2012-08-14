%% hough to lines %%

load('matlab_io/input_hough.mat')
threshold = ceil(0.3*max(H(:)));
hoodsize = ceil(size(H)/10);
hoodsize = hoodsize - mod(hoodsize,2) + [1 1];
P  = houghpeaks(H, 4, 'threshold', threshold, ...
                       'NHoodSize', hoodsize);
save('matlab_io/output_peaks.mat');
system('chown jens matlab_io/output_peaks.mat');


% plots
% imshow(H,[], 'InitialMagnification','fit');
% plot(P(:,2), P(:,1),'s','color','white');
