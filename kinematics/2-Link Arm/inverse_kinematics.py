import numpy as np

def inverse_kinematics(x, y, l1=1.0, l2=1.0):
    """
    Compute inverse kinematics for a 2-link planar robotic arm.

    Parameters:
    - x, y (float): target end-effector position
    - l1, l2 (float): lengths of the two links

    Returns:
    - (theta1, theta2): joint angles in radians
    """

    # Distance from origin to target
    r = np.sqrt(x**2 + y**2)

    # Check reachability
    if r > (l1 + l2) or r < abs(l1 - l2):
        raise ValueError("Target is out of reach for the arm!")

    # Law of cosines for theta2
    cos_theta2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    theta2 = np.arccos(cos_theta2)  # Elbow "down" solution

    # Two possible solutions: elbow-up and elbow-down
    theta2_alt = -theta2

    # Solve for theta1
    k1 = l1 + l2 * np.cos(theta2)
    k2 = l2 * np.sin(theta2)
    theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)

    # Alternate solution
    k1_alt = l1 + l2 * np.cos(theta2_alt)
    k2_alt = l2 * np.sin(theta2_alt)
    theta1_alt = np.arctan2(y, x) - np.arctan2(k2_alt, k1_alt)

    return (theta1, theta2), (theta1_alt, theta2_alt)


if __name__ == "__main__":
    # Example target position
    x, y = 0.7, 1.0
    l1, l2 = 1.0, 1.0

    try:
        (theta1, theta2), (theta1_alt, theta2_alt) = inverse_kinematics(x, y, l1, l2)
        print("Solution 1 (elbow-down):")
        print(f"θ1 = {np.degrees(theta1):.2f}°, θ2 = {np.degrees(theta2):.2f}°")

        print("\nSolution 2 (elbow-up):")
        print(f"θ1 = {np.degrees(theta1_alt):.2f}°, θ2 = {np.degrees(theta2_alt):.2f}°")
    except ValueError as e:
        print(e)
