% calculate Ad_T
function Ad = adt(T)
    R = T(1:3, 1:3);
    p = T(1:3, end);

    P = [0, -p(3), p(2); p(3), 0, -p(1); -p(2), p(1), 0];
    Ad = [R, zeros(3); P * R, R];
end
