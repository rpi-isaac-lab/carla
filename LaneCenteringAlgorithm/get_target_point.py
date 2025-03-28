# -*- coding: utf-8 -*-

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
        A point with a positive x-coordinate where the lookahead circle intersects the path.
        If multiple such points exist, returns the first one encountered along the path.
        Returns None if no valid intersection is found.

    Notes
    -----
    - This function checks each line segment of the polyline for intersections with the lookahead circle.
    - Only intersection points that lie within the segment (not on the extended line) and are in front of the vehicle (x > 0) are considered valid.
    - The first valid intersection in the order of the path is returned as the target point.
    """
    intersections = []

    # Loop over all consecutive point pairs (line segments) in the polyline
    for j in range(len(polyline) - 1):
        pt1 = polyline[j]
        pt2 = polyline[j + 1]

        # Find intersection(s) between the current segment and the lookahead circle
        segment_intersections = circle_line_segment_intersection(
            (0, 0),            # center of the lookahead circle
            lookahead,         # radius of the lookahead circle
            pt1, pt2,          # endpoints of the current segment
            full_line=False    # restrict to segment only (not infinite line)
        )
        intersections += segment_intersections

    # Filter out points that are behind the vehicle (x <= 0)
    filtered = [p for p in intersections if p[0] > 0]

    # Return the first valid point in path order, or None if none exist
    if len(filtered) == 0:
        return None
    return filtered[0]

