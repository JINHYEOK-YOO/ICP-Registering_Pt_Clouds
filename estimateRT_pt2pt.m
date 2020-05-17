function [R,t] = estimateRT_pt2pt(pts,ptsMoved)
    % Step 1
    % Centroid calculation
    cPts = mean(pts);
    cPtsMov = mean(ptsMoved);
    % Step 2
    % Move points to origin, we just substract the mean to each coordinate
    PtsC = pts - cPts;
    PtsMovC = ptsMoved - cPtsMov;
    % Step 3
    % Find the covariance between the point clouds
    covariance = PtsC' * PtsMovC;
    % Step 4 
    % Find the optimal rotation, (matrixR)
    [U,S,V] = svd(covariance);
    R = V * U';
    % Step 5
    % Find the translation t
    t = cPts - cPtsMov * R;
end