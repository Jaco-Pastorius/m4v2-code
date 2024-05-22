function f = f(in1, in2, w, d, l, varphi, m, g, J, Jinv)
    z = in1(1);
    dz = in1(2);
    phi = in1(3);
    th = in1(4);
    psi = in1(5);
    ox = in1(6);
    oy = in1(7);
    oz = in1(8);

    f1 = in2(1);
    f2 = in2(2);
    f3 = in2(3);
    f4 = in2(4);

    wrench = get_wrench_from_forces([f1;f2;f3;f4],w, d, l, varphi);

    tau = [wrench(1); wrench(2); wrench(3)];
    o = [ox; oy; oz];
    R = eulerzyx2rot(phi, th, psi);
    F = -tau(1) / (w + d * cos(varphi));

    fB = sym(zeros(3,1));
    fB(2) = F * sin(varphi);
    fB(3) = -wrench(4) * cos(varphi);

    % z is pointing down dynamics
    pdd = (1 / m) * R * fB + [0; 0; g];
    chid = getEinvZYX(phi, th, psi) * R * o;
    od = Jinv * (tau - cross(o, J * o));

    f = [dz; pdd(3); chid(3); chid(2); chid(1) ; od(1); od(2); od(3)];
end
