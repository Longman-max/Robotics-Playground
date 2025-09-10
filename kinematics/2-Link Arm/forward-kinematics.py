import numpy as np

def forward_kinematics(theta1, theta2, l1=1.0, l2=1.0):
    """
    Compute forward kinematics for a 2-link planar robotic arm.

    Parameters:
    - theta1 (float): Angle of first joint in radians
    - theta2 (float): Angle of second joint in radians
    - l1 (float): Length of first link
    - l2 (float): Length of second link

    Returns:
    - (x, y): Coordinates of the end-effector
    """

    # Position of first joint
    x1 = l1 * np.cos(theta1)
    y1 = l1 * np.sin(theta1)

    # Position of end-effector
    x2 = x1 + l2 * np.cos(theta1 + theta2)
    y2 = y1 + l2 * np.sin(theta1 + theta2)

    return (x2, y2)


if __name__ == "__main__":
    # Example: 2 joints with 45° each, links of length 1
    theta1 = np.deg2rad(45)
    theta2 = np.deg2rad(45)
    l1, l2 = 1.0, 1.0

    x, y = forward_kinematics(theta1, theta2, l1, l2)
    print(f"End-effector position: x = {x:.3f}, y = {y:.3f}")
