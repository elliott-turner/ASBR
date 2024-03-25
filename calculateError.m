% calculate angular and linear error between two transformation matrices
function e = calculateError(T1, T2)
    velMat = tform(se3(inv(T1) * T2));
    e = adt(T1) * [velMat(3, 2); velMat(1, 3); velMat(2, 1); velMat(1: 3, 4)];
end
