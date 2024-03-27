import numpy as np

def find_rotation_translation(points_A, points_B):
    """
    Find rotation matrix and translation vector from points in system A to points in system B.
    
    Args:
    points_A: List of points in system A.
    points_B: List of corresponding points in system B.
    
    Returns:
    rotation_matrix: Rotation matrix from system A to system B.
    translation_vector: Translation vector from system A to system B.
    """
    assert len(points_A) == len(points_B), "Number of points in system A and B should be the same."

    # Convert lists to numpy arrays
    A = np.array(points_A)
    B = np.array(points_B)

    # Find centroid of each point cloud
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)

    # Subtract centroids to get centered point clouds
    centered_A = A - centroid_A
    centered_B = B - centroid_B

    # Compute covariance matrix
    H = np.dot(centered_A.T, centered_B)

    # Perform SVD on covariance matrix
    U, _, Vt = np.linalg.svd(H)

    # Compute rotation matrix
    R = np.dot(Vt.T, U.T)

    # Handle special reflection case
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = np.dot(Vt.T, U.T)

    # Compute translation vector
    t = centroid_B - np.dot(R, centroid_A)

    return R, t

# Example usage:
points_A = [[1, 2, 3], [4, 5, 6], [7, 8, 9]]
points_B = [[3, 2, 1], [6, 5, 4], [9, 8, 7]]

rotation_matrix, translation_vector = find_rotation_translation(points_A, points_B)
print("Rotation Matrix:")
print(rotation_matrix)
print("Translation Vector:")
print(translation_vector)


def find_corresponding_point(point_A, rotation_matrix, translation_vector):
    """
    Find the corresponding point in system B for a point in system A.
    
    Args:
    point_A: Point in system A.
    rotation_matrix: Rotation matrix from system A to system B.
    translation_vector: Translation vector from system A to system B.
    
    Returns:
    point_B: Corresponding point in system B.
    """
    point_A = np.array(point_A)
    
    # Apply rotation and translation to point A
    point_B = np.dot(rotation_matrix, point_A) + translation_vector
    
    return point_B

# Example usage:
point_A = [1, 2, 3]  # Example point in system A
point_B = find_corresponding_point(point_A, rotation_matrix, translation_vector)
print("Corresponding point in system B:")
print(point_B)
