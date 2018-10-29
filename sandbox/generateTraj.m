load('data/waypoints.mat');
t= 0:14;
tq = 0:0.1:14;
slope0 = 0; slopeF = 0;
xq = spline(t, [slope0; x; slopeF], tq);
yq = spline(t, [slope0; y; slopeF], tq);
figure; plot(x, y, 'o', xq, yq, ':.');