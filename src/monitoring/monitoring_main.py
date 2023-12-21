import ctypes
import sys
import os

# Get the current script's directory
current_dir = os.path.dirname(os.path.abspath(__file__))

# Get the parent directory
parent_dir = os.path.dirname(current_dir)

# Add the parent directory to the sys.path
sys.path.append(parent_dir)

from windows.animation_window import *
from .robotat_3pi_Python import *
from windows.map_coordinates import inverse_change_coordinates

### ------------ END OF IMPORTS ------------

# This solves scaling issues for the independent pygame window
ctypes.windll.user32.SetProcessDPIAware()


pictures_dir = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "pictures"
)


class RobotAnimation:
    """
    This class manages the monitoring functions to include them in another script as in a GUI. It holds the methods for one marker monitoring and two markers monitoring. Methods to hold n-markers should be handled in this class.
    """

    def __init__(self):
        # Connection to the server
        self.robotat = robotat_connect()
        self.robotat.recv(2048)

        # Initialize arrays to display and save data
        self.x_data = []
        self.y_data = []
        self.theta_data = []
        self.x_results_raw = []
        self.y_results_raw = []

        # First we need to create the object that represents and updates the position and rotation of the Pololu img, this is the case for two markers
        self.characters_multiple = [
            (os.path.join(pictures_dir, "pololu_img_x.png"), 0, 0, 0),
            (os.path.join(pictures_dir, "pololu_img_x.png"), 100, 100, 0),
        ]

        # And this is the case for one robot only
        self.character = (os.path.join(pictures_dir, "pololu_img_x.png"), 0, 0, 0)

        self.animation_window = None

    def get_and_process_data_multiple(self, marker, representation):
        """
        This function is used to get the real time data with the Robotat functions in robotat_3pi_python.py module and to store them together to use them later in the display.

        Attributes:
        ---------------
        marker: list
            It receives the list of markers to monitor, tests were done going from the highest ID to the lowest.
        representation: str
            This refers to the option of EULER ANGLES or QUATERNIONS

        Returns:
        ------------
        x_data, y_data, theta_data:
            Data of the two markers, this data is used to be mapped before displayed on the Pygame window.
        """
        x_data = []
        y_data = []
        theta_data = []

        for pose_data_list in get_pose_continuous_multiple(
            self.robotat, marker, representation, max_attempts=5
        ):
            if pose_data_list is not None:
                for marker_data in pose_data_list:
                    x_vals_real_time = [marker_data[0]]
                    y_vals_real_time = [marker_data[1]]
                    theta_vals_real_time = [marker_data[5]]

                    x_data.append(x_vals_real_time)
                    y_data.append(y_vals_real_time)
                    theta_data.append(theta_vals_real_time)

        return x_data, y_data, theta_data

    def get_and_process_data(self, marker):
        """
        This function is used to get the real time data with the Robotat functions in robotat_3pi_python.py module. The angle is modified to display the correct baesd on the position of the Optitrack marker. This same values should be added in the multiple monitoring to handle any marker.

        Attributes:
        ---------------
        marker: list
            It receives the number of marker to monitor.
        representation: str
            This refers to the option of EULER ANGLES or QUATERNIONS

        Returns:
        ------------
        x_data, y_data, theta_data:
            Data of the marker, this data is used to be mapped before displayed on the Pygame window.
        """
        try:
            for pose_data in get_pose_continuous(
                self.robotat, [marker], "eulxyz", max_attempts=5
            ):
                x_vals_real_time = [pose_data[0][0]]
                y_vals_real_time = [pose_data[0][1]]
                theta_vals_real_time = [pose_data[0][5]]
                print(marker)
                if marker == 1:
                    theta_vals_real_time = [pose_data[0][5]]  # done
                elif marker == 2:
                    theta_vals_real_time = [pose_data[0][5] - 40]  # done
                elif marker == 3:
                    theta_vals_real_time = [pose_data[0][5] - 90]
                elif marker == 4:
                    theta_vals_real_time = [pose_data[0][5] - 140]
                elif marker == 5:
                    theta_vals_real_time = [pose_data[0][5] + 175]
                elif marker == 6:
                    theta_vals_real_time = [pose_data[0][5] - 145]
                elif marker == 7:
                    theta_vals_real_time = [pose_data[0][5] + 90]  # done
                elif marker == 8:
                    theta_vals_real_time = [pose_data[0][5] - 10]  # done
                elif marker == 9:
                    theta_vals_real_time = [pose_data[0][5] - 80]  # done
                elif marker == 13:
                    theta_vals_real_time = [pose_data[0][5] + 40]  # done
                elif marker == 14:
                    theta_vals_real_time = [pose_data[0][5] + 20]  # done
                self.x_data.append(x_vals_real_time)
                self.y_data.append(y_vals_real_time)
                self.theta_data.append(theta_vals_real_time)
                break
        except Exception as e:
            print(f"ERROR: {e}")

        return x_vals_real_time, y_vals_real_time, theta_vals_real_time

    def map_data_multiple(
        self, x_vals_real_time, y_vals_real_time, theta_vals_real_time
    ):
        """
        This function takes the captured data and modifies it for the display on the Pygame window.

        Attributes:
        ---------------
        x_vals_real_time, y_vals_real_time, theta_vals_real_time
            Data captured in real time (Optitrack coordinate and measure units)

        Returns:
        --------------
        x_vals_display_robot, y_vals_display_robot, theta_vals_display_robot
            The same data scaled, ready to display in the Pygame window
        """
        x_vals_display_robot = []
        y_vals_display_robot = []
        theta_vals_display_robot = []

        for marker_x, marker_y, marker_theta in zip(
            x_vals_real_time, y_vals_real_time, theta_vals_real_time
        ):
            x_mapped = []
            y_mapped = []
            theta_mapped = []

            for x, y, theta in zip(marker_x, marker_y, marker_theta):
                x_raw, y_raw = x, y
                x_new_val, y_new_val = inverse_change_coordinates(
                    x_raw * 100, y_raw * 100, 960, 760
                )
                theta_new_val = theta + 180

                x_mapped.append(x_new_val)
                y_mapped.append(y_new_val)
                theta_mapped.append(theta_new_val)

            x_vals_display_robot.append(x_mapped)
            y_vals_display_robot.append(y_mapped)
            theta_vals_display_robot.append(theta_mapped)

        return x_vals_display_robot, y_vals_display_robot, theta_vals_display_robot

    def map_data(self, x_vals_real_time, y_vals_real_time, theta_vals_real_time):
        """
        This function takes the captured data and modifies it for the display on the Pygame window.

        Attributes:
        ---------------
        x_vals_real_time, y_vals_real_time, theta_vals_real_time
            Data captured in real time (Optitrack coordinate and measure units)

        Returns:
        --------------
        x_vals_display_robot, y_vals_display_robot, theta_vals_display_robot
            The same data scaled, ready to display in the Pygame window
        """
        x_vals_display_robot = []
        y_vals_display_robot = []
        theta_vals_display_robot = []

        for x, y, theta in zip(
            x_vals_real_time, y_vals_real_time, theta_vals_real_time
        ):
            x_raw, y_raw = x, y
            x_new_val, y_new_val = inverse_change_coordinates(
                x_raw * 100, y_raw * 100, 960, 760
            )

            theta_new_val = theta  # theta_val, not just theta

            x_vals_display_robot.append(x_new_val)
            y_vals_display_robot.append(y_new_val)
            theta_vals_display_robot.append(theta_new_val)
            self.x_results_raw.append(x_raw)
            self.y_results_raw.append(y_raw)

        # Wrap the final arrays in a list
        x_vals_display_robot = [x_vals_display_robot]
        y_vals_display_robot = [y_vals_display_robot]
        theta_vals_display_robot = [theta_vals_display_robot]

        return x_vals_display_robot, y_vals_display_robot, theta_vals_display_robot

    def get_data_multiple(self, tag1, tag2):
        """
        This function takes the previous definitions for the case of multiple markers.
        """
        (
            x_vals_real_time,
            y_vals_real_time,
            theta_vals_real_time,
        ) = self.get_and_process_data_multiple([tag1, tag2], "eulxyz")

        (
            x_vals_display_robot,
            y_vals_display_robot,
            theta_vals_display_robot,
        ) = self.map_data_multiple(
            x_vals_real_time, y_vals_real_time, theta_vals_real_time
        )

        return (
            x_vals_display_robot,
            y_vals_display_robot,
            theta_vals_display_robot,
            x_vals_real_time,
            y_vals_real_time,
            theta_vals_real_time,
        )

    def get_data(self, tag):
        """
        This function takes the previous definitions for the case of one marker.
        """
        (
            x_vals_real_time,
            y_vals_real_time,
            theta_vals_real_time,
        ) = self.get_and_process_data(tag)
        (
            x_vals_display_robot,
            y_vals_display_robot,
            theta_vals_display_robot,
        ) = self.map_data(x_vals_real_time, y_vals_real_time, theta_vals_real_time)
        return (
            x_vals_display_robot,
            y_vals_display_robot,
            theta_vals_display_robot,
            x_vals_real_time,
            y_vals_real_time,
            theta_vals_real_time,
        )

    def setup_animation_window(self, filename, tag):
        """
        This initializes the monitoring window calling the child class of the animation window module. It adds the characters to the window and leaves everything set up for running the monitoring.
        """
        self.animation_window = py_game_monitoring(
            850, 960, lambda: self.get_data(tag), filename
        )
        self.animation_window.add_robot_character(*self.character)
        self.animation_window.initialize()

    def setup_animation_window_multiple(self, filename, tag1, tag2):
        """
        This initializes the monitoring window calling the child class of the animation window module. It adds the characters to the window and leaves everything set up for running the monitoring.
        """
        self.animation_window = py_game_monitoring_multiple(
            850, 960, lambda: self.get_data_multiple(tag1, tag2), filename
        )
        for character in self.characters_multiple:
            self.animation_window.add_robot_character(*character)
        self.animation_window.initialize()

    def animation_function(self, flag):
        """
        It starts the animation with a run flag, this run flag can be started with a user input for example. When the animation starts the monitoring (capturing data) begins, and when the Pygame window is closed the data transaction is finished.
        """
        pygame.init()
        while flag:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    flag = False  # Set the flag to False to exit the loop
                    pygame.quit()
            self.animation_window.start_animation()
