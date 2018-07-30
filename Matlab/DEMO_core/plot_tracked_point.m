function plot_tracked_point(ipRelations, cam)

% assign parameters
L = 1;


% plot tracked points on the image plane
ipRelationsNum = size(ipRelations, 2);
for k = 1:ipRelationsNum
    u1 = (cam.K(1,1) * ipRelations{k}.x + cam.K(1,3)) / (2^(L-1));
    v1 = (cam.K(2,2) * ipRelations{k}.y + cam.K(2,3)) / (2^(L-1));
    u2 = (cam.K(1,1) * ipRelations{k}.z + cam.K(1,3)) / (2^(L-1));
    v2 = (cam.K(2,2) * ipRelations{k}.h + cam.K(2,3)) / (2^(L-1));
    
    % current point feature
    if (abs(ipRelations{k}.v) < 0.5)             % no depth point
        plot(u2, v2, 'rs');
    elseif (abs(ipRelations{k}.v - 1) < 0.5)   % depth point from RGB-D
        plot(u2, v2, 'gs');
    end
    
    % point feature tracking result
    plot([u2 u1], [v2 v1], 'y');
end


end