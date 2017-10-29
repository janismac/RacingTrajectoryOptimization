function track = generate_track_polygons( checkpoints )

    track = struct;
    
    % Interlace left and right track boundary points
    track.vertices = nan(2, 2*length(checkpoints));
    track.vertices(:,1:2:end) = [checkpoints.left];
    track.vertices(:,2:2:end) = [checkpoints.right];

    % Generate initial set of polygons (quadrilaterals) that cover the track
    for i = 1:length(checkpoints)
        track.polygons(i).vertex_indices = mod1(2*i + (-1:2), size(track.vertices,2));
    end

    track = merge_polygons(track);
    track = add_overlaps(track);    
    track = add_forward_directions(track, checkpoints);
    
end