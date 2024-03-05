function out = getE_ZYX_dot(phi, th, psi, dphi, dth, dpsi)
    cz = cos(psi); sz = sin(psi);
    cy = cos(th); sy = sin(th);
    out = sym(zeros(3,3));
    out(1,1) = 0;
    out(1,2) = -cz * dpsi;
    out(1,3) = -sy * cz * dth - cy * sz * dpsi;
    out(2,1) = 0;
    out(2,2) = -sz * dpsi;
    out(2,3) = -sy * cz * dth - cy * sz * dpsi;
    out(3,1) = 0;
    out(3,2) = 0;
    out(3,3) = -cy * dth;
end
