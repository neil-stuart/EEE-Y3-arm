import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import pandas as pd

from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression

class PredictionModel:
    def __init__(self):
        self.timestamped_position_df = pd.DataFrame(columns=['timestamp_ms', 'X', 'Y', 'Z'])

    # Predict the value of XYS at a timestamp. 
    def predict_future_vals(self, timestamp=None):
        # Implement prediction logic here based on stored data
        pass
    
    def add_data(self, txyz):
        if len(txyz) != 4:
            raise ValueError("Data should have 4 elements: timestamp_ms, X, Y, Z")
        data_dict = {'timestamp_ms': txyz[0], 'X': txyz[1], 'Y': txyz[2], 'Z': txyz[3]}
        new_data = pd.DataFrame(data_dict, index=[0])
        self.timestamped_position_df = pd.concat([self.timestamped_position_df,new_data], ignore_index=True)


class QuadraticPredictionModel(PredictionModel):
    def __init__(self):
        super().__init__()
    
    def predict_future_vals(self, timestamp):
        # Extracting features and target from stored data
        X = self.timestamped_position_df['timestamp_ms'].values.reshape(-1, 1)
        y = self.timestamped_position_df[['X', 'Y', 'scale_depth']].values
        
        # Fitting quadratic regression model separately for each feature
        quadratic_features = PolynomialFeatures(degree=2)
        X_quadratic = quadratic_features.fit_transform(X)
        
        predicted_values = []
        for i in range(y.shape[1]):
            model = LinearRegression()
            model.fit(X_quadratic, y[:, i])
            future_timestamp = np.array([[timestamp]])
            future_quadratic = quadratic_features.transform(future_timestamp)
            predicted_val = model.predict(future_quadratic)
            predicted_values.append(predicted_val[0])
        
        return predicted_values



class ControlsGenerator(Node):
    def __init__(self):
        super().__init__('controls_generator')

        # current_time = self.get_clock().now()
        
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'xys',
            self.calculate_controls,
            10)
        


        self.model = PredictionModel()

        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Float32MultiArray, 'control', 10)
        self.i = 0

    def calculate_controls(self, msg):
        xyz = msg.data
        self.i += 1

        # Use the xy position and s 'scale' like z distance to determine direction arm needs to move 
        # Publish to control topic
        xyz.insert(0, self.get_clock().now().nanoseconds)
        txyz = xyz
        self.model.add_data(txyz)

        # Predict future xys values at timedelta.
        self.model.predict_future_vals()

        # If predicted values fall within a range the arm can reach. 


        # Then find motor angles for that mid-point. 


        # Define parameters
        controls = Float32MultiArray()
        controls.data = [0.0,0.0,0.0,0.0,0.0,0.0] #Lets say 5 motors, giving angles between 0 and mazx angle
        
        
        
        self.publisher_.publish(controls)
        
        if self.i == 30:
            self.get_logger().info("Current Desired Position: " + ' '.join(str(round(i,2)) for i in controls.data))
            self.i = 0
        

def main(args=None):
    rclpy.init(args=args)

    controls_generator = ControlsGenerator()

    rclpy.spin(controls_generator)

    controls_generator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()