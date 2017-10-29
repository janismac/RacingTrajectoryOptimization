function track = add_forward_directions(track, checkpoints)
    n_checkpoints = length(checkpoints);
    forward_vectors = [checkpoints.forward_vector];
    for i = 1:length(track.polygons)
        A = track.polygons(i).A;
        b = track.polygons(i).b;
        
        included_checkpoints_selector = all(A*[checkpoints.center] < repmat(b,1,n_checkpoints));
        track.polygons(i).forward_direction = mean(forward_vectors(:, included_checkpoints_selector),2);
    end
end

