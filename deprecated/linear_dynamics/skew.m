function out = skew(v)
    out = zeros(3,3);
    out(1,1) = 0;
    out(1,2) = -v(3);
    out(1,3) = v(2);
    out(2,1) = v(3);
    out(2,2) = 0;
    out(2,3) = -v(1);
    out(3,1) = -v(2);
    out(3,2) = v(1);
    out(3,3) = 0;
end