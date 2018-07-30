function [matchingNum, goodMatchingIdx] = inThresforPoseAll(ipRelations, xi_21, cam, distance)

% re-assign 6 DoF camera motion
tx = xi_21(1);
ty = xi_21(2);
tz = xi_21(3);
phi = xi_21(4);
theta = xi_21(5);
psi = xi_21(6);


% reprojection error distance of each point
ipRelationsNum = size(ipRelations,2);
distanceEachPoint = zeros(1,ipRelationsNum);
for i = 1:ipRelationsNum
    
    ipr = ipRelations{i};
    
    uv1_u = [ipr.x; ipr.y];
    uv2_u = [ipr.z; ipr.h];
    u1 = uv1_u(1);
    v1 = uv1_u(2);
    u2 = uv2_u(1);
    v2 = uv2_u(2);
    
    if (abs(ipr.v) < 0.5) % no depth point
        
        y2 = f_nodepth([tx; ty; tz; phi; theta; psi; u2; v2; 0; 0; 0; u1; v1]);
        distanceEachPoint(i) = abs(y2);
        
    elseif (abs(ipr.v - 1) < 0.5 || abs(ipr.v - 2) < 0.5) % with depth point
        
        X_1 = u1 * ipr.s;
        Y_1 = v1 * ipr.s;
        Z_1 = ipr.s;
        
        y3 = f_depth_1([tx; ty; tz; phi; theta; psi; u2; v2; X_1; Y_1; Z_1; u1; v1]);
        y4 = f_depth_2([tx; ty; tz; phi; theta; psi; u2; v2; X_1; Y_1; Z_1; u1; v1]);
        distanceEachPoint(i) = (abs(y3) + abs(y4))/2;
    end
end


% determine inlier or not
matchingNum = sum(distanceEachPoint <= distance);
goodMatchingIdx = find(distanceEachPoint <= distance);


end
