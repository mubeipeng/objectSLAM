clear all;
%% generate 2D objects
truth_objects = rand(15,2)*10;
truth_objects(:,3) = mod(1:15, 5);
%truth_objects(:,3)=[1:5,1:5,1:5];

%% plot 2D objects
figure; hold on; set(0,'DefaultLineMarkerSize',10)
for i=1:length(truth_objects)
    switch truth_objects(i,3)
        case 0 
            plot(truth_objects(i,1),truth_objects(i,2),'bo','MarkerFaceColor','b');            
        case 1
            plot(truth_objects(i,1),truth_objects(i,2),'rd','MarkerFaceColor','r');
        case 2
            plot(truth_objects(i,1),truth_objects(i,2),'ms','MarkerFaceColor','m');
        case 3
            plot(truth_objects(i,1),truth_objects(i,2),'g^','MarkerFaceColor',[0.2 1 0.2]);
        case 4
            plot(truth_objects(i,1),truth_objects(i,2),'yp','MarkerFaceColor',[1 1 0.3],...
                                'MarkerSize',15);
    end
end

%% draw trajectory
[x,y] = ginput();

%% generate odometry
%truth_traj=[];

% for i=2:length(x)
%     l = norm([x(i)-x(i-1) y(i)-y(i-1)]);
%     t=0:0.1:l;
%     xq=interp1([0 l],[x(i-1) x(i)],t,'spline');
%     yq=interp1([0 l],[y(i-1) y(i)],t,'spline');
%     truth_traj = [truth_traj; [xq' yq']];
% end

t= 1:length(x);
tq = 1:0.1:length(x);
slope0 = 0; slopeF = 0;
xq = spline(t, [slope0; x; slopeF], tq);
yq = spline(t, [slope0; y; slopeF], tq);
%figure; plot(x, y, 'o', xq, yq, ':.');
truth_traj = [xq' yq'];

for i=1:length(truth_traj)-1
    odoms(i,3)=atan2(truth_traj(i+1,2)-truth_traj(i,2),truth_traj(i+1,1)-truth_traj(i,1));
end

truth_traj = truth_traj(1:end-1,:);

odoms(end,3)=odoms(end-1,3);
odoms(:,1:2) = truth_traj(1:end,1:2);

truth_traj(1:end,3) = odoms(:,3);
truth_traj(end, 3) = truth_traj(end,3);

%%
for i=2:length(odoms)
    R = [cos(odoms(i,3)),sin(odoms(i,3)); -sin(odoms(i,3)) cos(odoms(i,3))];
    node_edge.dpos(:,i)= R'*(odoms(i,1:2)-odoms(i-1,1:2))';
    node_edge.dtheta(i) = odoms(i,3)-odoms(i-1,3);
end

%% generate object measurements
FOV=4; AOV=60/180*pi;
lm_edge.id1=[];
lm_edge.id2=[];
lm_edge.dpos=[];
lm_edge.label=[];
for i=2:length(odoms)
    dx=truth_objects(:,1)-odoms(i,1);
    dy=truth_objects(:,2)-odoms(i,2);
    dtheta = mod(atan2(dy,dx)-odoms(i,3)+pi,2*pi)-pi;
    R = [cos(odoms(i,3)),sin(odoms(i,3)); -sin(odoms(i,3)) cos(odoms(i,3))];
    idx = find((dx.^2+dy.^2)<FOV^2 & abs(dtheta)<AOV);
    dpos = R'*[dx';dy'];
    lm_edge.id1=[lm_edge.id1 repmat(i,1,length(idx))-1];
    lm_edge.id2=[lm_edge.id2 idx'];
    lm_edge.dpos=[lm_edge.dpos dpos(:,idx)];
    % Handle lm_edge label being 0.
    % ALternative will be to handle lm_edge being 0 when it is being used
    % as the index in the processor object
    
    lm_edge.label=[lm_edge.label truth_objects(idx,3)'];
end

lm_edge.label = 5*(lm_edge.label == 0) + lm_edge.label;

%% save data as mat file
save('data/my_simulation.mat', 'AOV', 'FOV', 'lm_edge', 'node_edge', 'truth_objects', 'truth_traj');