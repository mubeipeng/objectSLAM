addpath('include/');

%% load data and add noise
load('data/simulation.mat');
lm_edge.dpos = lm_edge.dpos + randn(2,1098)*0.1;
node_edge.dpos = node_edge.dpos+ randn(size(node_edge.dpos))*0.02;
node_edge.dtheta = node_edge.dtheta + randn(size(node_edge.dtheta))*0.01;

%% plot dataset
fig = figure;
set(fig,'Position', [100, 100, 400, 300]);
set(fig,'Units','Inches');
pos = get(fig,'Position');
set(fig,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])

plot(truth_traj(:,1),truth_traj(:,2),'k');
hold on; 
        
label={'tajectory'};
for i=1:5
    idx = truth_objects(:,3)==i-1;
    if sum(idx)>0
        label{end+1}=['class ' num2str(i)];
        switch i
        case 1
            plot(truth_objects(idx,1),truth_objects(idx,2),'bo','MarkerFaceColor','b');            
        case 2
            plot(truth_objects(idx,1),truth_objects(idx,2),'rd','MarkerFaceColor','r');
        case 3
            plot(truth_objects(idx,1),truth_objects(idx,2),'ms','MarkerFaceColor','m');
        case 4
            plot(truth_objects(idx,1),truth_objects(idx,2),'g^','MarkerFaceColor',[0.2 1 0.2]);
        case 5
            plot(truth_objects(idx,1),truth_objects(idx,2),'yp','MarkerFaceColor',[1 0.7 0.3],...
                                        'MarkerSize',15);
        end               
   end
end
axis equal; axis off;
legend(label);
        
%% non-parametric
pr = Processer();
pr = pr.setupobjects(node_edge,lm_edge);
pr = pr.optimize(5);
% [Iodom_NP, Iobj_NP]=pr.computeEntropy();
[Eodom_NP, Eobj_NP]=pr.computeError(truth_traj',truth_objects');
pr.plot();

%% frame based
pr_frame = Processer();
pr_frame = pr_frame.setupobjects(node_edge,lm_edge);
% [Iodom_frame, Iobj_frame]=pr_frame.computeEntropy();
[Eodom_frame, Eobj_frame]=pr_frame.computeError(truth_traj',truth_objects');
pr_frame.plot();

%% open loop
dp_alpha = 0.2:0.1:0.9;
for i=1:length(dp_alpha)
    pr_OL = Processer();
    pr_OL.DP_alpha=dp_alpha(i);
    pr_OL = pr_OL.setupobjects(node_edge,lm_edge);
    pr_OL = pr_OL.OpenLoop;
%     pr_OL.plot();
    % [Iodom_OL, Iobj_OL]=pr_OL.computeEntropy();
    [Eodom_OL, Eobj_OL]=pr_OL.computeError(truth_traj',truth_objects');
    err_obj_ol(i)=mean(sqrt(Eobj_OL));
    no_obj_ol(i)=length(pr_OL.objects);
end
pr_OL.plot();

%% robust slam
for i=1:length(dp_alpha)
    pr_Robust = Processer();
    pr_Robust.DP_alpha=dp_alpha(i);
    pr_Robust = pr_Robust.setupobjects(node_edge,lm_edge);
    pr_Robust = pr_Robust.RobustSlam;
    pr_Robust.write_isam('isam_input.txt');
    !isam/bin/isam isam_input.txt -W isam_output.txt -B
    pr_Robust = pr_Robust.read_isam('isam_output.txt');
%     pr_Robust.plot();
    % [Iodom_PL, Iobj_PL]=pr_PL.computeEntropy();
    [Eodom_Robust, Eobj_Robust]=pr_Robust.computeError(truth_traj',truth_objects');
    err_obj_robust(i)=mean(sqrt(Eobj_Robust));
    no_obj_robust(i)=length(pr_Robust.objects);
end
pr_Robust.plot();


%%
algo = {'NP pose graph'; 'Open Loop'; 'Robust SLAM'; 'Frame by Frame'};
mean_odom = mean( sqrt([Eodom_NP; Eodom_OL; Eodom_Robust; Eodom_frame]),2);
cum_odom = sum( sqrt([Eodom_NP; Eodom_OL; Eodom_Robust; Eodom_frame]),2);
no_msts = sum( [pr.measurements.obj_id; pr_OL.measurements.obj_id;...
    pr_Robust.measurements.obj_id; pr_frame.measurements.obj_id]>0,2);
no_obj = [length(pr.objects); length(pr_OL.objects); length(pr_Robust.objects); length(pr_frame.objects)];
err_obj = [mean(sqrt(Eobj_NP)); mean(sqrt(Eobj_OL)); mean(sqrt(Eodom_Robust)); mean(sqrt(Eobj_frame))];
T = table(mean_odom,cum_odom,no_msts,no_obj,err_obj,...
    'RowNames',algo);
%%
set(0,'DefaultLineMarkerSize',10)
semilogx(no_obj(1), err_obj(1),'ko','MarkerFaceColor','k');hold on;
semilogx(no_obj_ol, err_obj_ol,'b^','MarkerFaceColor','b');
semilogx(no_obj_robust, err_obj_robust,'rs','MarkerFaceColor','r');
semilogx(no_obj(4), err_obj(4),'mp','MarkerFaceColor','m');
semilogx(15, 0,'gd','MarkerFaceColor',[0 0.8 0]);
xlabel('number of objects');ylabel error;
legend('NP-Graph','OL','R-SLAM','FbF','Ground Truth');

%%
semilogy(cumsum(sqrt(Eodom_NP(1:20:end))),'k-o');hold on;
semilogy(cumsum(sqrt(Eodom_OL(1:20:end))),'b-^');
semilogy(cumsum(sqrt(Eodom_Robust(1:20:end))),'r-s');
semilogy(cumsum(sqrt(Eodom_frame(1:20:end))),'m-p');
xlabel time;ylabel 'cumulative trajectory error';
legend('NP-Graph','OL','R-SLAM','FbF');

