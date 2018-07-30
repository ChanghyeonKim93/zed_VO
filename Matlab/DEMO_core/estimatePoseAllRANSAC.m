function [xi_21_final, ipRelationsInliers] = estimatePoseAllRANSAC(ipRelations, cam, optsMWO)

% initialize RANSAC model parameters
totalPointNum = size(ipRelations,2);
samplePointNum = 6;
ransacMaxIterNum = 1000;
ransacIterNum = 50;
ransacIterCnt = 0;

maxMatchingNum = 0;
maxMatchingIdx = [];
distance = 0.002;


% do RANSAC with 3-DoF translational motion
while (true)
    
    % sample 3 feature points
    [sampleIdx] = randsample(totalPointNum, samplePointNum);
    ipRelationsSample = cell(1,6);
    ipRelationsSample{1} = ipRelations{sampleIdx(1)};
    ipRelationsSample{2} = ipRelations{sampleIdx(2)};
    ipRelationsSample{3} = ipRelations{sampleIdx(3)};
    ipRelationsSample{4} = ipRelations{sampleIdx(4)};
    ipRelationsSample{5} = ipRelations{sampleIdx(5)};
    ipRelationsSample{6} = ipRelations{sampleIdx(6)};
    
    
    % estimate translational motion with 3 feature points
    xi_21_candidate = estimatePoseAll(ipRelationsSample, cam, optsMWO);
    
    
    % check number of inliers
    [matchingNum, goodMatchingIdx] = inThresforPoseAll(ipRelations, xi_21_candidate, cam, distance);
    
    
    % save the large consensus set
    if (matchingNum >= maxMatchingNum)
        maxMatchingNum = matchingNum;
        maxMatchingIdx = goodMatchingIdx;
        xi_21_best = xi_21_candidate;
        
        % calculate the number of iterations (http://en.wikipedia.org/wiki/RANSAC)
        matchingRatio = matchingNum / totalPointNum;
        ransacIterNum = ceil(log(0.01)/log(1-(matchingRatio)^samplePointNum));
    end
    
    ransacIterCnt = ransacIterCnt + 1;
    if (ransacIterCnt >= ransacIterNum || ransacIterCnt >= ransacMaxIterNum)
        break;
    end
end


% refine translation with RANSAC inliers
ipRelationsInliers = cell(1,maxMatchingNum);
for k = 1:maxMatchingNum
    ipRelationsInliers{k} = ipRelations{maxMatchingIdx(k)};
end
xi_21_final = estimatePoseAll(ipRelationsInliers, cam, optsMWO);


end






