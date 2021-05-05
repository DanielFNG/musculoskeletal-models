function R = computeRotationMatrix(a, b)
% Computes the rotation matrix R which rotates the vector a on to the vector b.

    % First we normalise the vectors a and b.
    a = a/norm(a);
    b = b/norm(b);
    
    % Next, compute the cross product, and its skew-symmetric matrix.
    v = cross(a, b);
    v_x = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
    
    % Compute the cosine of the angle between the vectors.
    c = dot(a, b);
    
    % Throw an error if we have anti-parallel vectors.
    if c == -1
        error('Cannot compute for anti-parallel vectors.');
    end

    % Compute rotation matrix.
    R = eye(3) + v_x + v_x*v_x*(1/(1 + c));

end