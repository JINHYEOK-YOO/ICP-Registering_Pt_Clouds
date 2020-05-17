function [R,t] = ICP(pts,ptsMoved)
    % Find the corresponding closest point
    [Idx,D] = knnsearch(pts,ptsMoved);
    corres = pts(Idx,:);
    
    % Calculate the registration parameters R and t given point correspondences
    [R,t] = estimateRT_pt2pt(corres,ptsMoved);
end