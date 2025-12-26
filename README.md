# Cobot-UR5-Analysis
UR5 6-DOF cobot kinematics in MATLAB: FK via DH parameters, IK with Newton-Raphson solver, trajectory planning, and interactive GUI. Achieves 98.6% IK success with sub-mm accuracy and real-time 3D visualization.

## Author

**Krish Santoki**
- MS Robotics @ Northeastern University
- Email: santoki.k@northeastern.edu
- LinkedIn: https://www.linkedin.com/in/krish-santoki/
- GitHub: @krishsantoki (https://github.com/krishsantoki)


## Overview
This project implements a complete kinematic analysis framework for the Universal Robots UR5 collaborative robot. The implementation includes forward kinematics using Denavit-Hartenberg (DH) parameters, inverse kinematics with Newton-Raphson iterative solver, trajectory planning for letter-K path following, and an interactive GUI controller for real-time task-space manipulation.

**Key Application**: Demonstrates robotic motion planning and control concepts applicable to industrial automation, pick-and-place operations, and research in collaborative robotics.

## Features
- **Forward Kinematics**: Analytical FK using standard DH convention with homogeneous transformation matrices
- **Inverse Kinematics**: Newton-Raphson solver with damped least-squares Jacobian regularization (λ=0.05)
- **Trajectory Planning**: Letter-K path generation with 71 waypoints and 10mm spacing
- **Interactive GUI**: Real-time task-space control with translation (±100mm) and rotation (±30°) buttons
- **3D Visualization**: Stick-figure robot animation with coordinate frames and path history
- **High Accuracy**: 98.6% IK success rate with sub-millimeter position accuracy
- **Fast Convergence**: Average 5.7 iterations per IK solution

## Usage
- Generates 71 waypoints forming the letter "K" in YZ plane at X=0.3m
- Solves inverse kinematics for each waypoint using Newton-Raphson method
- Animates the robot tracing the letter with 3D stick-figure visualization
- Displays joint angles, iteration counts, and success metrics
- Shows position/rotation errors for validation

### Interactive GUI Controller
Launch the interactive controller:
```matlab
% Open and run the interactive controller
interactive_new.m
```

**Controls:**
- **+X / -X**: Move end-effector ±100mm along X-axis
- **+Y / -Y**: Move end-effector ±100mm along Y-axis
- **+Z / -Z**: Move end-effector ±100mm along Z-axis
- **+Rx / -Rx**: Rotate end-effector ±30° about X-axis
- **+Ry / -Ry**: Rotate end-effector ±30° about Y-axis
- **+Rz / -Rz**: Rotate end-effector ±30° about Z-axis
- **HOME**: Return to starting configuration (arm pointing toward +X)
- **CLEAR PATH**: Reset path history visualization

**Features:**
- Real-time IK solving with visual feedback
- Path history tracking (magenta line)
- Live joint angle display
- Status messages for successful moves or IK failures
- 3D robot visualization with coordinate frames (RGB = XYZ)

**Note:** Both MATLAB scripts are self-contained with all necessary functions (DH matrices, FK, IK, Jacobian, rotation conversions) defined within each file.

## Technical Details

### UR5 DH Parameters
| Joint | a (m) | d (m) | α (rad) | α (deg) |
|-------|-------|-------|---------|---------|
| 1 | 0.00000 | 0.08916 | π/2 | 90 |
| 2 | -0.42500 | 0.00000 | 0 | 0 |
| 3 | -0.39225 | 0.00000 | 0 | 0 |
| 4 | 0.00000 | 0.10915 | π/2 | 90 |
| 5 | 0.00000 | 0.09465 | -π/2 | -90 |
| 6 | 0.00000 | 0.08230 | 0 | 0 |

**Robot Specifications:**
- Degrees of Freedom: 6 (all revolute joints)
- Reach: 850 mm
- Payload: 5 kg
- Repeatability: ±0.1 mm

### Forward Kinematics
End-effector pose computed via sequential transformation:
```
T₀⁶ = T₀¹ × T¹² × T²³ × T³⁴ × T⁴⁵ × T⁵⁶
```

Each transformation follows standard DH convention:
```matlab
T = [cos(θ)  -sin(θ)cos(α)   sin(θ)sin(α)   a·cos(θ)
     sin(θ)   cos(θ)cos(α)  -cos(θ)sin(α)   a·sin(θ)
       0         sin(α)         cos(α)          d
       0           0              0             1    ]
```

### Inverse Kinematics Algorithm
**Newton-Raphson with Damped Least Squares:**

1. **Initialize**: q = q₀ (previous solution or initial guess)
2. **Compute FK**: T_current = FK(q)
3. **Calculate Error**: 
   - Position error: Δp = p_target - p_current
   - Rotation error: Δr = axis-angle from R_error = R_target × R_currentᵀ
   - Combined: e = [Δp; Δr] (6×1 vector)
4. **Compute Jacobian**: J = jacobian_ur5(q) (6×6 geometric Jacobian)
5. **Damped Update**: Δq = (JᵀJ + λ²I)⁻¹Jᵀe where λ = 0.05
6. **Update Joints**: q = q + Δq
7. **Check Convergence**: If ||Δp|| < 10⁻⁵ m and ||Δr|| < 10⁻⁵ rad, stop
8. **Repeat**: Maximum 100 iterations

**Geometric Jacobian:**
```
For each joint i:
  Jv,i = zi-1 × (pe - pi-1)  [linear velocity component]
  Jω,i = zi-1                 [angular velocity component]
```

### Trajectory Generation
**Letter-K Specifications:**
- **Plane**: YZ plane at constant X = 0.3 m
- **Center Point**: [0.3, 0.25, 0.5] m
- **Segments**:
  1. Vertical stem: 200mm (±100mm along Z)
  2. Upper diagonal: 100mm at +45° 
  3. Lower diagonal: 100mm at -45°
- **Waypoints**: 71 total points with 10mm spacing
- **Drawing Sequence**: 
  - Middle → Top → Middle → Bottom → Middle → Upper diagonal → Middle → Lower diagonal

**End-Effector Orientation:** Constant throughout trajectory (tool pointing in +X direction)

## Results

### Letter-K Trajectory Performance
| Metric | Value |
|--------|-------|
| Total Waypoints | 71 |
| Successful IK Solutions | 70 (98.6%) |
| Average Iterations per Point | 5.7 |
| Total Computation Time | 0.021 seconds |
| Average Time per Point | 0.0003 seconds |
| Position Accuracy | < 10⁻⁶ mm |
| Rotation Accuracy | < 10⁻⁶ degrees |

### Validation
**Sample Joint Angles (degrees) for Key Points:**
- **Top of K** (z=0.6m): [-150.27, -30.67, -67.03, 97.70, 119.73, -90.00]
- **Middle** (z=0.5m): [-150.27, -17.04, -86.73, 103.77, 119.73, -90.00]
- **Bottom** (z=0.4m): [-150.27, -3.96, -103.48, 107.44, 119.73, -90.00]

**Forward Kinematics Verification:**
- At home position (all joints = 0): End-effector at [-0.817, -0.191, -0.005] m
- Tool orientation verified: X-axis pointing in robot +X direction

### Design Decisions
1. **Self-Contained Scripts**: All functions embedded in main scripts for easy sharing and execution
2. **Damped Least Squares**: λ=0.05 provides numerical stability near singularities
3. **Initial Guess Strategy**: Uses previous solution for trajectory following to improve convergence
4. **Joint Wrapping**: All joint angles wrapped to [-π, π] for consistency

### Future Improvements
- Add joint limit enforcement (typical UR5 limits: ±360° for joints 1-5, ±180° for joint 6)
- Implement analytical IK for faster computation (closed-form solution exists for UR5)
- Add velocity-level control for smoother trajectory execution
- Include workspace boundary visualization
- Implement multiple IK solution selection

## References

1. **UR5 Technical Specifications**: Universal Robots UR5 datasheet
2. **DH Parameters**: [Universal Robots DH Parameters](https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/)
3. **Denavit-Hartenberg Convention**: J.J. Craig, "Introduction to Robotics: Mechanics and Control"
4. **Inverse Kinematics**: B. Siciliano et al., "Robotics: Modelling, Planning and Control"
5. **Damped Least Squares**: S. Chiaverini et al., "Singularity-Robust Task-Priority Redundancy Resolution"
