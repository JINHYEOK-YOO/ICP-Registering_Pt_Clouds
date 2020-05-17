function f = rigidTransform(pts,R,t)
    f = (pts * R) + t;
end