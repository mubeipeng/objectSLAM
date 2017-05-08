classdef ImageProcessor
properties
    opts
    model
    rpn_net
    fast_rcnn_net
    classes
    thres
    Xmultiplier = (ones(480,1)*(1:640)-324.4524060051391)/670.9182132265033; 
    Ymultiplier = ((1:480)'*ones(1,640)-192.8382650244068)/671.4906633201279;
end
methods

    %%    
    function obj = ImageProcessor(caffe_dir)
        obj = obj.init_path(caffe_dir);
        
        %% caffe.set_device(0);
        caffe.set_mode_gpu();
        obj.opts.per_nms_topN           = 6000;
        obj.opts.nms_overlap_thres      = 0.7;
        obj.opts.after_nms_topN         = 300;
        obj.opts.use_gpu                = true;
        obj.opts.test_scales            = 480;        
        
        obj = obj.init_net();
        
        %% init object detect
        obj.classes = obj.model.classes;
        obj.thres = [0.9,... //1 laptop
                    0.9,... //2 bus
                    0.6,... //3 helmet
                    0.9,... //4 bench
                    0.9,... //5 motorcycle
                    0.5,... //6 keyboard
                    0.9,... //7 bicycle
                    1.00,... //8 table
                    0.6,... //9 tv
                    0.9,... //10 backpack
                    0.3,... //11 flower-pot
                    0.2,... //12 traffic light
                    0.99,... //13 lamp
                    0.3,... //14 cup
                    0.99,... //15 dog
                    0.6,... //16 car
                    0.8,... //17 chair
                    0.9,... //18 bowl
                    0.9,... //19 person
                    0.9,... //20 sofa
                    ];             
    end
    
    function obj = init_path(obj,caffe_dir)
        addpath(caffe_dir);
        addpath('../include/fast_rcnn', '../include/rpn', '../include/nms','../include/utils');
        a=gpuArray(1); clear a;
    end
    
    function obj = init_net(obj)
        model_dir = '../faster_rcnn_customImageNet';
%         model_dir = '../faster_rcnn_Indoor_CaffeNet';
        ld        = load(fullfile(model_dir, 'model'));
        obj.model     = ld.proposal_detection_model;
        clear ld;
    
        obj.model.proposal_net_def  = fullfile(model_dir, obj.model.proposal_net_def);
        obj.model.proposal_net      = fullfile(model_dir, obj.model.proposal_net);
        obj.model.detection_net_def = fullfile(model_dir, obj.model.detection_net_def);
        obj.model.detection_net     = fullfile(model_dir, obj.model.detection_net);

        obj.model.conf_proposal.test_scales = obj.opts.test_scales;
        obj.model.conf_detection.test_scales = obj.opts.test_scales;
        obj.model.conf_proposal.image_means = gpuArray(obj.model.conf_proposal.image_means);
        obj.model.conf_detection.image_means = gpuArray(obj.model.conf_detection.image_means);

        obj.rpn_net = caffe.Net(obj.model.proposal_net_def, 'test');
        obj.rpn_net.copy_from(obj.model.proposal_net);
        obj.fast_rcnn_net = caffe.Net(obj.model.detection_net_def, 'test');
        obj.fast_rcnn_net.copy_from(obj.model.detection_net);
        
        %% -------------------- WARM UP --------------------
        % the first run will be slower; use an empty image to warm up

        for j = 1:2 % we warm up 2 times
            im = uint8(ones(375, 500, 3)*128);
            if obj.opts.use_gpu
                im = gpuArray(im);
            end
            [boxes, scores]             = proposal_im_detect(obj.model.conf_proposal, obj.rpn_net, im);
            aboxes                      = boxes_filter([boxes, scores], obj.opts.per_nms_topN, obj.opts.nms_overlap_thres, obj.opts.after_nms_topN, obj.opts.use_gpu);
            if obj.model.is_share_feature
                [boxes, scores]             = fast_rcnn_conv_feat_detect(obj.model.conf_detection, obj.fast_rcnn_net, im, ...
                    obj.rpn_net.blobs(obj.model.last_shared_output_blob_name), ...
                    aboxes(:, 1:4), obj.opts.after_nms_topN);
            else
                [boxes, scores]             = fast_rcnn_im_detect(obj.model.conf_detection, obj.fast_rcnn_net, im, ...
                    aboxes(:, 1:4), obj.opts.after_nms_topN);
            end
        end
    end
    
    function objects = process(obj,image)      
        
        %% detect object        
        im = gpuArray(image);
        [boxes, scores]             = proposal_im_detect(obj.model.conf_proposal, obj.rpn_net, im);
        aboxes                      = boxes_filter([boxes, scores], obj.opts.per_nms_topN, obj.opts.nms_overlap_thres, obj.opts.after_nms_topN, obj.opts.use_gpu);
        [boxes, scores]             = fast_rcnn_conv_feat_detect(obj.model.conf_detection, obj.fast_rcnn_net, im, ...
                                        obj.rpn_net.blobs(obj.model.last_shared_output_blob_name),...
                                        aboxes(:, 1:4), obj.opts.after_nms_topN);
        %% refine boxes
        objects = [];
        for i = 1:length(obj.classes)
            boxes_cell = [boxes(:, (1+(i-1)*4):(i*4)), scores(:, i)];
            boxes_cell = boxes_cell(nms(boxes_cell, 0.3), :);        
            I = boxes_cell(:, 5) >= obj.thres(i);
%             I = boxes_cell(:, 5) >= 0.3;
            boxes_cell = boxes_cell(I, :);
            boxes_cell(:,6)=i;
            objects = [objects; boxes_cell];
        end        
    end    
    
    function objects = createPC(obj,image,depth,boxes)
        r=image(:,:,1);g=image(:,:,2);b=image(:,:,3);
        Z=depth; X=Z.*obj.Xmultiplier; Y=Z.*obj.Ymultiplier; 
        validI = Z>0 & Z.^2<36;
        
        objects=[];
        for k=1:size(boxes,1)
            % generate point cloud
            box = round(boxes(k,1:4));
            mask = false(480,640);
            mask(box(2):box(4),box(1):box(3)) = validI(box(2):box(4),box(1):box(3));
            d = prctile(Z(mask),30);
            mask = mask & Z>d-0.3 & Z<d+0.3;
            if sum(mask(:))<1000
                continue;
            end
            
            rgb=[r(mask) g(mask) b(mask)];
%             location = [X(mask) Y(mask) Z(mask)]*R_odom_cam';
            location = [X(mask) Y(mask) Z(mask)];

            object.pc = pointCloud(location(:,1:3),'color',rgb);
            m = mean(location);
%             T = [eye(3) -m'; zeros(1,3), 1];
%             object.pc = pctransform(pc,affine3d(T'));
            object.box=box;
            object.label = boxes(k,6);
            object.score = boxes(k,5);
            object.T = [eye(3) m'; zeros(1,3), 1];
            object.size = max([object.pc.XLimits(2)-object.pc.XLimits(1), object.pc.YLimits(2)-object.pc.YLimits(1)]);
            objects = [objects object];
        end
    end
    
    function im = showboxes(obj,im,boxes)
        colors=lines;
        for i = 1:size(boxes,1)
            box = boxes(i, 1:4);
            im = insertShape(im, 'Rectangle',RectLTRB2LTWH(box), 'LineWidth', 4,'Color',colors(boxes(i,6),:));
            im = insertText(im,[double(box(1))+2, double(box(2))],[obj.classes{boxes(i,6)} num2str(boxes(i,5))]);
            %rectangle('Position', RectLTRB2LTWH(box), 'LineWidth', linewidth, 'EdgeColor', colors{boxes(i,6)});
            %label = sprintf('%s(%d)', legends{boxes(i,6)}, i);
            %text(double(box(1))+2, double(box(2)), label, 'BackgroundColor', 'w');
        end
    end    
end
end