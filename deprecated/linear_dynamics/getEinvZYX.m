function out = getEinvZYX(phi, th, psi)
    cz = cos(psi); sz = sin(psi);
    cy = cos(th); sy = sin(th);
    out = sym(zeros(3,3));
    out(1,1) = cz * sy / cy;
    out(1,2) = sy * sz / cy;
    out(1,3) = 1;
    out(2,1) = -sz;
    out(2,2) = cz;
    out(2,3) = 0;
    out(3,1) = cz / cy;
    out(3,2) = sz / cy;
    out(3,3) = 0;
end
