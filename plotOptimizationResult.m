function plotOptimizationResult(p, x, controller_output)

    clf
    set(gcf,'color',[1 1 1]);    
    hold on
    
    left_points = [p.checkpoints(:).left];
    right_points = [p.checkpoints(:).right];
    forward_vectors = [p.checkpoints.forward_vector];
        
    % Draw track area
    pts=[fliplr(right_points) right_points(:,end) left_points left_points(:,1)];
    fill(pts(1,:), pts(2,:),[1 1 1]*.8,'EdgeAlpha',0)
        
    % Draw track outline with extra width for the vehicle
    width = .6;    
    normals = width*[0 -1;1 0]*forward_vectors;
    left_points = left_points + normals;
    right_points = right_points - normals;
    plot([left_points(1,:) left_points(1,1)],[left_points(2,:) left_points(2,1)],'k','LineWidth',1);
    plot([right_points(1,:) right_points(1,1)],[right_points(2,:) right_points(2,1)],'k','LineWidth',1);
        
    % Draw planned trajectory
    plot(x(1,:),x(2,:),'.-r','MarkerSize',7);    
    
    % Vehicle box
    d = x(3:4,1);
    d = d / norm(d);
    R = [d [-d(2); d(1)]];
    vehicleRectangle = R*[1 0;0 .5]*[1 1 -1 -1;1 -1 -1 1]+repmat(x(1:2,1),1,4);
    fill(vehicleRectangle(1,:),vehicleRectangle(2,:),[0 0 0]);
    daspect([1 1 1])
    
    % Constraints for the last trajectory point
    if strcmp(p.name, 'SL') 
        last_checkpoint_index = controller_output.checkpoint_indices(end);
        last_checkpoint = p.checkpoints(last_checkpoint_index);
        L = 30;
        tangent_left = [(last_checkpoint.left + L* last_checkpoint.forward_vector) (last_checkpoint.left - L* last_checkpoint.forward_vector)];
        tangent_right = [(last_checkpoint.right + L* last_checkpoint.forward_vector) (last_checkpoint.right - L* last_checkpoint.forward_vector)];
        plot(tangent_left(1,:), tangent_left(2,:),'k--','LineWidth',1)
        plot(tangent_right(1,:), tangent_right(2,:),'k--','LineWidth',1)
    elseif strcmp(p.name, 'SCR') 
        poly_idx = controller_output.track_polygon_indices(end);
        poly = p.track_polygons(poly_idx);
        vertices = con2vert(poly.A, poly.b);
        K = convhull(vertices(:,1), vertices(:,2));
        vertices = vertices(K,:);
        plot(vertices(:,1), vertices(:,2),'k','LineWidth',1.5);
    end
    
    % Center track in figure, add padding
    bounds = [min([left_points right_points]');max([left_points right_points]')];    
    xlim(bounds(:,1))
    ylim(bounds(:,2))
    axis off
    set(gca, 'Position', [0 0 1 1])
    xlim(mean(xlim) + diff(xlim) * [-1 1] * 0.6)
    ylim(mean(ylim) + diff(ylim) * [-1 1] * 0.6)    
    daspect([1 1 1])
        
end