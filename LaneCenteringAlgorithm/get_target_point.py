def get_target_point(lookahead, polyline):
    """
    Determines the target point for the Pure Pursuit controller.

    Parameters
    ----------
    lookahead : float
        The radius of the lookahead circle, centered at the vehicle's rear axle (assumed to be at (0, 0)).
    
    polyline : array_like, shape (M, 2)
        A list of 2D points representing the path to follow. Each consecutive pair defines a segment of the path.

    Returns
    -------
    target_point : numpy array, shape (2,)
        A point where the lookahead circle intersects the path, lies in front of the vehicle (x > 0),
        and has a direction angle |arctan(y/x)| > π/4 (i.e., at least 45° off from straight ahead).
        Returns the first such point found in path order.
        Returns None if no valid intersection exists.
    """
    intersections = []

    # Loop through each segment in the path
    for j in range(len(polyline) - 1):
        pt1 = polyline[j]
        pt2 = polyline[j + 1]

        # Get intersection points between this segment and the lookahead circle
        segment_intersections = circle_line_segment_intersection(
            (0, 0),        # center of the lookahead circle (vehicle rear axle)
            lookahead,     # radius of the lookahead circle
            pt1, pt2,      # endpoints of the current segment
            full_line=False  # only consider the finite line segment, not the infinite line
        )
        intersections += segment_intersections

    # Filter intersection points:
    # 1. Must be in front of the vehicle (x > 0)
    # 2. Must have angle |arctan(y/x)| > π/4 (i.e., more than 45 degrees off from forward)
    filtered = [
        p for p in intersections
        if p[0] > 0 and abs(np.arctan2(p[1], p[0])) > np.pi / 4
    ]

    # Return the first valid target point if any
    if len(filtered) == 0:
        return None
    return filtered[0]
