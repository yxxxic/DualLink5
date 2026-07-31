function distance = segmentDistance(p1, p2, q1, q2, tolerance)
if segmentsIntersect(p1, p2, q1, q2, tolerance)
    distance = 0;
    return
end

distance = min([ ...
    pointDistance(p1, q1, q2, tolerance), ...
    pointDistance(p2, q1, q2, tolerance), ...
    pointDistance(q1, p1, p2, tolerance), ...
    pointDistance(q2, p1, p2, tolerance)]);
end

function distance = pointDistance(point, a, b, tolerance)
ab = b - a;
lengthSquared = dot(ab, ab);
if lengthSquared <= tolerance^2
    distance = norm(point - a);
    return
end

projection = dot(point - a, ab) / lengthSquared;
projection = max(0, min(1, projection));
distance = norm(point - (a + projection * ab));
end
