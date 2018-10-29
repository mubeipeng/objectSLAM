load('data/waypoints.mat');
t= 1:length(x);
tq = 1:0.1:length(x);
slope0 = 0; slopeF = 0;
xq = spline(t, [slope0; x; slopeF], tq);
yq = spline(t, [slope0; y; slopeF], tq);
figure; plot(x, y, 'o', xq, yq, ':.');