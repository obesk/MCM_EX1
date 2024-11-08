function R = AngleAxisToRot(h,theta)
    
    skew_h = [
        0, -h(3), h(2);
        h(3), 0, -h(1);
        -h(2), h(1), 0;
    ];
    
    I = eye(3);
    R = I + sin(theta) * skew_h + (1-cos(theta)) * (skew_h ^ 2);

end