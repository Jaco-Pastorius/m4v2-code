function wrench = get_wrench_from_forces(forces, w, d, l, varphi)
    u = [1, 1, 1, 1; -1, 1, -1, 1; -1, -1, 1, 1; 1, -1, -1, 1] * forces;

    W = [0, w + d * cos(varphi), 0, 0;
         0, 0, l * cos(varphi), 0;
         0, 0, 0, l * sin(varphi);
         1, 0, 0, 0];
    
    wrench = W * u;
end
