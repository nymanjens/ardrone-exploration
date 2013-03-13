%% hough to lines %%

load('matlab_io/input_hough.mat')
threshold = ceil(0.1*max(H(:)));
%hoodsize = ceil(size(H)/3);
hoodsize = ceil(size(H)/7); % original value
hoodsize = hoodsize - mod(hoodsize,2) + [1 1];
P  = houghpeaks(H, 7, 'threshold', threshold, ...
                       'NHoodSize', hoodsize);
save('matlab_io/output_peaks.mat');
system('chown jens matlab_io/output_peaks.mat');


% plots
% imshow(H,[], 'InitialMagnification','fit');
% plot(P(:,2), P(:,1),'s','color','white');
