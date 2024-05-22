function out = getE_ZYX(phi, th, psi)
    cz = cos(psi); sz = sin(psi);
    cy = cos(th); sy = sin(th);
    out = sym(zeros(3,3));
    out(1,1) = 0;
    out(1,2) = -sz;
    out(1,3) = cy * cz;
    out(2,1) = 0;
    out(2,2) = cz;
    out(2,3) = cy * sz;
    out(3,1) = 1;
    out(3,2) = 0;
    out(3,3) = -sy;
end
