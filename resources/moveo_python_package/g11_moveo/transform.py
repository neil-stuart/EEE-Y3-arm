import numpy as np
import csv


class TransformPoint():
    def __init__(self, file_path="./resources/calibrate/points.csv"):

        self.calibA, self.calibB = self.__load_calibration_points(file_path)
        self.R, self.t = self.__find_rotation_translation()

    def __find_rotation_translation(self):
        """
        Find rotation matrix and translation vector from points in system A to points in system B.
        
        Args:
        points_A: List of points in system A.
        self.calibB: List of corresponding points in system B.
        
        Returns:
        rotation_matrix: Rotation matrix from system A to system B.
        translation_vector: Translation vector from system A to system B.
        """
        assert len(self.calibA) == len(self.calibB), "Number of points in system A and B should be the same."

        # Convert lists to numpy arrays
        A = np.array(self.calibA)
        B = np.array(self.calibB)

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



    def __load_calibration_points(self, file_path):
        points_A = []
        self.calibB = []

        with open(file_path, newline='') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                # Extract tracked and arm positions from each row
                tracked_pos = [float(row['trackedX']), float(row['trackedY']), float(row['trackedZ'])]
                arm_pos = [float(row['armX']), float(row['armY']), float(row['armZ'])]

                # Append to respective lists
                points_A.append(tracked_pos)
                self.calibB.append(arm_pos)

        return points_A, self.calibB


    def find_corresponding_point(self, point):
        """
        Find the corresponding point in system B for a point in system A.
        
        Args:
        point_A: Point in system A.
        rotation_matrix: Rotation matrix from system A to system B.
        translation_vector: Translation vector from system A to system B.
        
        Returns:
        point_B: Corresponding point in system B.
        """
        point = np.array(point)
        
        # Apply rotation and translation to point A
        point_B = np.dot(self.R, point) + self.t
        
        return point_B

