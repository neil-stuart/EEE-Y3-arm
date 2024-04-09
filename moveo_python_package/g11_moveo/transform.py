import numpy as np
import csv


class TransformPoint():
    def __init__(self, system_a_points, system_b_points):
        """
        Class fior the 3rd year project to facilitate tranformaing a point from system A to a point in system B.
        e.g. Camera point to robotic arm point. Takes a list of points in system A and a corresponding lsit of points in system B.
        """
        self.calibA, self.calibB = system_a_points, system_b_points
        self.A = self.__find_rotation_translation()

    def __find_rotation_translation(self):
        assert len(self.calibA) == len(self.calibB), "Number of points in system A and B should be the same."

        # Pad the data with ones, so that our transformation can do translations too
        pad = lambda x: np.hstack([x, np.ones((x.shape[0], 1))])
        X = pad(self.calibA)
        Y = pad(self.calibB)

        # Solve the least squares problem X * A = Y
        # to find our transformation matrix A
        A, res, rank, s = np.linalg.lstsq(X, Y)    

        return A


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
        pad = lambda x: np.hstack([x, np.ones((x.shape[0], 1))])
        unpad = lambda x: x[:,:-1]
        
        transform = lambda x: unpad(np.dot(pad(x), self.A))
        
        return transform(point)

