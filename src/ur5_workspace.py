import numpy as np
import matplotlib.pyplot as plt
from urdfpy import URDF

# Load UR5 URDF file
ur5_urdf = URDF.load("/home/farid/Qontrol_Assignment/urdf/ur5_arm.urdf") 

# Extract joint limits (min, max for each joint)
joint_limits = np.array([[j.limit.lower, j.limit.upper] for j in ur5_urdf.joints if j.limit])

n_joints = joint_limits.shape[0]

n_samples = 5000


joint_angles = np.random.uniform(
    joint_limits[:, 0],  # Lower limits
    joint_limits[:, 1],  # Upper limits
    (n_samples, n_joints)
)


end_effector_positions = np.array([
    ur5_urdf.link_fk(cfg)[ur5_urdf.links[-1]][0:3, 3]
    for cfg in joint_angles
])


x, y, z = end_effector_positions[:, 0], end_effector_positions[:, 1], end_effector_positions[:, 2]

# Plot the workspace in 3D
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='3d')

ax.scatter(x, y, z, s=1, alpha=0.5, label="UR5 Workspace")

ax.set_xlabel("X-axis (m)")
ax.set_ylabel("Y-axis (m)")
ax.set_zlabel("Z-axis (m)")
ax.set_title("Non-Convex Workspace of UR5 Robot")
ax.legend()
plt.show()

