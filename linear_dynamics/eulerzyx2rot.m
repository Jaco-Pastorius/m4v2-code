function R = eulerzyx2rot(phi, th, psi)
    R = sym(zeros(3,3));
    cx = cos(phi); cy = cos(th); cz = cos(psi);
    sx = sin(phi); sy = sin(th); sz = sin(psi);
    R(1,1) = cy*cz;
    R(1,2) = cz*sx*sy - cx*sz;
    R(1,3) = sx*sz + cx*cz*sy;
    R(2,1) = cy*sz;
    R(2,2) = cx*cz + sx*sy*sz;
    R(2,3) = cx*sy*sz - cz*sx;
    R(3,1) = -sy;
    R(3,2) = cy*sx;
    R(3,3) = cx*cy;
end
