function im = showboxes(im, boxes, legends)
% Draw bounding boxes on top of an image.
%   showboxes(im, boxes)
%
% -------------------------------------------------------

% fix_width = 800;
if isa(im, 'gpuArray')
    im = gather(im);
end
% imsz = size(im);
% scale = fix_width / imsz(2);
% im = imresize(im, scale);

% axis image;
% axis off;
% set(gcf, 'Color', 'white');

valid_boxes_num = size(boxes,1);
if valid_boxes_num > 0
%     boxes(:,1:4) = boxes(:, 1:4) * scale;

    colors_candidate = colormap('hsv');
    colors_candidate = colors_candidate(1:(floor(size(colors_candidate, 1)/20)):end, :);
    colors_candidate = mat2cell(colors_candidate, ones(size(colors_candidate, 1), 1))';
    colors = colors_candidate;            

    for i = 1:valid_boxes_num
        box = boxes(i, 1:4);
        im = insertShape(im, 'Rectangle',RectLTRB2LTWH(box), 'LineWidth', 4,'Color',colors{boxes(i,6)});
        im = insertText(im,[double(box(1))+2, double(box(2))],[legends{boxes(i,6)} num2str(boxes(i,5))]);
        %rectangle('Position', RectLTRB2LTWH(box), 'LineWidth', linewidth, 'EdgeColor', colors{boxes(i,6)});
        %label = sprintf('%s(%d)', legends{boxes(i,6)}, i);
        %text(double(box(1))+2, double(box(2)), label, 'BackgroundColor', 'w');
    end
end
end

function [ rectsLTWH ] = RectLTRB2LTWH( rectsLTRB )
%rects (l, t, r, b) to (l, t, w, h)

rectsLTWH = [rectsLTRB(:, 1), rectsLTRB(:, 2), rectsLTRB(:, 3)-rectsLTRB(:,1)+1, rectsLTRB(:,4)-rectsLTRB(2)+1];
end

