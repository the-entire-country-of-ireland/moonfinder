import numpy as np
import math 


def data_regularize(data, type="spherical", divs=10):
    limits = np.array([
        [min(data[:, 0]), max(data[:, 0])],
        [min(data[:, 1]), max(data[:, 1])],
        [min(data[:, 2]), max(data[:, 2])]])
        
    regularized = []

    if type == "cubic": # take mean from points in the cube
        
        X = np.linspace(*limits[0], num=divs)
        Y = np.linspace(*limits[1], num=divs)
        Z = np.linspace(*limits[2], num=divs)

        for i in range(divs-1):
            for j in range(divs-1):
                for k in range(divs-1):
                    points_in_sector = []
                    for point in data:
                        if (point[0] >= X[i] and point[0] < X[i+1] and
                                point[1] >= Y[j] and point[1] < Y[j+1] and
                                point[2] >= Z[k] and point[2] < Z[k+1]):
                            points_in_sector.append(point)
                    if len(points_in_sector) > 0:
                        regularized.append(np.mean(np.array(points_in_sector), axis=0))

    elif type == "spherical": #take mean from points in the sector
        divs_u = divs 
        divs_v = divs * 2

        center = np.array([
            0.5 * (limits[0, 0] + limits[0, 1]),
            0.5 * (limits[1, 0] + limits[1, 1]),
            0.5 * (limits[2, 0] + limits[2, 1])])
        d_c = data - center
    
        #spherical coordinates around center
        r_s = np.sqrt(d_c[:, 0]**2. + d_c[:, 1]**2. + d_c[:, 2]**2.)
        d_s = np.array([
            r_s,
            np.arccos(d_c[:, 2] / r_s),
            np.arctan2(d_c[:, 1], d_c[:, 0])]).T

        u = np.linspace(0, np.pi, num=divs_u)
        v = np.linspace(-np.pi, np.pi, num=divs_v)

        for i in range(divs_u - 1):
            for j in range(divs_v - 1):
                points_in_sector = []
                for k, point in enumerate(d_s):
                    if (point[1] >= u[i] and point[1] < u[i + 1] and
                            point[2] >= v[j] and point[2] < v[j + 1]):
                        points_in_sector.append(data[k])

                if len(points_in_sector) > 0:
                    regularized.append(np.mean(np.array(points_in_sector), axis=0))
# Other strategy of finding mean values in sectors
#                    p_sec = np.array(points_in_sector)
#                    R = np.mean(p_sec[:,0])
#                    U = (u[i] + u[i+1])*0.5
#                    V = (v[j] + v[j+1])*0.5
#                    x = R*math.sin(U)*math.cos(V)
#                    y = R*math.sin(U)*math.sin(V)
#                    z = R*math.cos(U)
#                    regularized.append(center + np.array([x,y,z]))
    return np.array(regularized)


# https://github.com/minillinim/ellipsoid
def ellipsoid_plot(center, radii, rotation, ax, plot_axes=False, cage_color='b', cage_alpha=0.2):
    """Plot an ellipsoid"""
        
    u = np.linspace(0.0, 2.0 * np.pi, 100)
    v = np.linspace(0.0, np.pi, 100)
    
    # cartesian coordinates that correspond to the spherical angles:
    x = radii[0] * np.outer(np.cos(u), np.sin(v))
    y = radii[1] * np.outer(np.sin(u), np.sin(v))
    z = radii[2] * np.outer(np.ones_like(u), np.cos(v))
    # rotate accordingly
    for i in range(len(x)):
        for j in range(len(x)):
            [x[i, j], y[i, j], z[i, j]] = np.dot([x[i, j], y[i, j], z[i, j]], rotation) + center

    if plot_axes:
        # make some purdy axes
        axes = np.array([[radii[0],0.0,0.0],
                         [0.0,radii[1],0.0],
                         [0.0,0.0,radii[2]]])
        # rotate accordingly
        for i in range(len(axes)):
            axes[i] = np.dot(axes[i], rotation)

        # plot axes
        for p in axes:
            X3 = np.linspace(-p[0], p[0], 100) + center[0]
            Y3 = np.linspace(-p[1], p[1], 100) + center[1]
            Z3 = np.linspace(-p[2], p[2], 100) + center[2]
            ax.plot(X3, Y3, Z3, color=cage_color)

    # plot ellipsoid
    ax.plot_wireframe(x, y, z,  rstride=4, cstride=4, color=cage_color, alpha=cage_alpha)


# http://www.mathworks.com/matlabcentral/fileexchange/24693-ellipsoid-fit
# for arbitrary axes
def fit_ellipsoid_standard(X, linsolve=np.linalg.solve):
    x = X[:, 0]
    y = X[:, 1]
    z = X[:, 2]
    D = np.array([x * x + y * y - 2 * z * z,
                 x * x + z * z - 2 * y * y,
                 2 * x * y,
                 2 * x * z,
                 2 * y * z,
                 2 * x,
                 2 * y,
                 2 * z,
                 1 - 0 * x])
    d2 = np.array(x * x + y * y + z * z).T # rhs for LLSQ
    lhs, rhs = D.dot(D.T), D.dot(d2)
    u = linsolve(lhs, rhs)
    a = np.array([u[0] + 1 * u[1] - 1])
    b = np.array([u[0] - 2 * u[1] - 1])
    c = np.array([u[1] - 2 * u[0] - 1])
    v = np.concatenate([a, b, c, u[2:]], axis=0).flatten()
    A = np.array([[v[0], v[3], v[4], v[6]],
                  [v[3], v[1], v[5], v[7]],
                  [v[4], v[5], v[2], v[8]],
                  [v[6], v[7], v[8], v[9]]])

    center = np.linalg.solve(- A[:3, :3], v[6:9])

    translation_matrix = np.eye(4)
    translation_matrix[3, :3] = center.T

    R = translation_matrix.dot(A).dot(translation_matrix.T)

    evals, evecs = np.linalg.eig(R[:3, :3] / -R[3, 3])
    evecs = evecs.T

    radii = np.sqrt(1. / np.abs(evals))
    radii *= np.sign(evals)

    return center, evecs, radii


import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def set_axes_equal(ax: plt.Axes):
    ax.set_box_aspect([1,1,1])
    limits = np.array([
        ax.get_xlim3d(),
        ax.get_ylim3d(),
        ax.get_zlim3d(),
    ])
    x, y, z = np.mean(limits, axis=1)
    radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))
    ax.set_xlim3d([x - radius, x + radius])
    ax.set_ylim3d([y - radius, y + radius])
    ax.set_zlim3d([z - radius, z + radius])


# http://www.cs.brandeis.edu/~cs155/Lecture_07_6.pdf
# Affine transformation from ellipsoid to unit sphere
def transform(data, center, evecs, radii):
    """
    Transform the data to fit a unit sphere.
    """
    # Center the data
    data = data - center

    # Rotate the data using the eigenvectors
    data = np.dot(data, evecs.T)

    # Scale the data using the radii
    data = data / radii
    
    # Rotate the data using the eigenvectors
    data = np.dot(data, evecs)

    return data


def sample_noisy_ellipsoid(center, R, radii, num_points=1000, noise_level=0.01):
    # Generate random points on a unit sphere
    phi = np.random.uniform(0, 2 * np.pi, num_points)
    costheta = np.random.uniform(-1, 1, num_points)

    theta = np.arccos(costheta)
    r = 1

    x = r * np.sin(theta) * np.cos(phi)
    y = r * np.sin(theta) * np.sin(phi)
    z = r * np.cos(theta)

    # Scale points to the ellipsoid
    points = np.vstack((x, y, z)).T
    points *= radii

    # Rotate and translate points
    points = np.dot(points, R.T)
    points += center

    # Add noise
    noise = np.random.normal(0, noise_level, points.shape)
    noisy_points = points + noise

    return noisy_points

import numpy as np
from scipy.optimize import linprog

def solve_l1(A, b):
    """
    Solve the least L1 norm problem: minimize ||A @ x - b||_1
    with an optional warm start.
    """
    m, n = A.shape

    # Create the linear programming problem
    # Minimize: sum of auxiliary variables (t)
    # Subject to: -t <= A @ x - b <= t
    c = np.hstack([np.zeros(n), np.ones(m)])  # Objective function: [0...0, 1...1]
    G = np.vstack([
        np.hstack([A, -np.eye(m)]),  # A @ x - t <= b
        np.hstack([-A, -np.eye(m)])  # -A @ x - t <= -b
    ])
    h = np.hstack([b, -b])  # Combine b and -b


    # Solve the linear programming problem
    result = linprog(c, A_ub=G, b_ub=h,
                     method='highs-ipm',
                     options={'presolve': True})

    if result.success:
        return result.x[:n]  # Return the solution x
    else:
        raise ValueError("L1 minimization failed: " + result.message)


import numpy as np
from scipy.optimize import least_squares

def ellipsoid_residuals(params, points):
    """
    Compute residuals for ellipsoid fitting.
    params: [cx, cy, cz, rx, ry, rz, q1, q2, q3, q4]
        - cx, cy, cz: Center of the ellipsoid
        - rx, ry, rz: Radii of the ellipsoid
        - q1, q2, q3, q4: Quaternion representing rotation
    points: Nx3 array of data points
    """
    # Extract parameters
    cx, cy, cz = params[:3]
    rx, ry, rz = params[3:6]
    q = params[6:]

    # Convert quaternion to rotation matrix
    R = quaternion_to_rotation_matrix(q)

    # Compute residuals
    residuals = []
    for point in points:
        # Translate point to ellipsoid frame
        p = point - np.array([cx, cy, cz])
        # Rotate point
        p_rot = R @ p
        # Compute scaled distance
        d = (p_rot[0] / rx)**2 + (p_rot[1] / ry)**2 + (p_rot[2] / rz)**2
        # Residual is distance to ellipsoid surface minus 1
        residuals.append(d - 1)
    return np.array(residuals)

def fit_ellipsoid_l2(points, params_init=None):
    """
    Fit an ellipsoid to a set of 3D points.
    points: Nx3 array of data points
    """
    # Initial guess
    center_init = np.mean(points, axis=0)
    radii_init = np.std(points, axis=0)
    quaternion_init = [1, 0, 0, 0]  # Identity rotation
    if params_init is None:
        params_init = np.hstack([center_init, radii_init, quaternion_init])

    # Define bounds for the parameters
    bounds = (
        [-100, -100, -100, 0, 0, 0, -1, -1, -1, -1],  # Lower bounds
        [100, 100, 100, 100, 100, 100, 1, 1, 1, 1]  # Upper bounds
    )

    # Optimize with bounds
    result = least_squares(ellipsoid_residuals, params_init, args=(points,), bounds=bounds)

    # Extract results
    cx, cy, cz = result.x[:3]
    rx, ry, rz = result.x[3:6]
    q = result.x[6:]
    R = quaternion_to_rotation_matrix(q)

    return np.array([cx, cy, cz]), R, np.array([rx, ry, rz])


def fit_ellipsoid_l1(points, params_init=None):
    """
    Fit an ellipsoid to a set of 3D points using L1 minimization.
    points: Nx3 array of data points
    """
    # Initial guess
    center_init = np.mean(points, axis=0)
    radii_init = np.std(points, axis=0)
    quaternion_init = [1, 0, 0, 0]  # Identity rotation
    if params_init is None:
        params_init = np.hstack([center_init, radii_init, quaternion_init])

    # Define the L1 loss function
    def l1_loss(params):
        residuals = ellipsoid_residuals(params, points)
        return np.sum(np.abs(residuals))  # L1 norm

    # Optimize using L1 minimization
    result = minimize(l1_loss, params_init, method='Powell')

    # Extract results
    cx, cy, cz = result.x[:3]
    rx, ry, rz = result.x[3:6]
    q = result.x[6:]
    R = quaternion_to_rotation_matrix(q)

    return np.array([cx, cy, cz]), R, np.array([rx, ry, rz])

def solution_to_params(center, R, radii):
    params_init = np.hstack([center, radii, rotation_matrix_to_quaternion(R)])
    return params_init


from scipy.optimize import minimize
import numpy as np

def rotation_matrix_to_quaternion(R):
    """
    Convert a 3x3 rotation matrix to a quaternion [q1, q2, q3, q4].
    The quaternion is in the form [w, x, y, z], where w is the scalar part.
    
    Parameters:
        R (numpy.ndarray): 3x3 rotation matrix.
    
    Returns:
        numpy.ndarray: Quaternion [w, x, y, z].
    """
    # Ensure the matrix is a valid rotation matrix
    assert R.shape == (3, 3), "Input must be a 3x3 matrix."
    
    # Compute the trace of the matrix
    trace = np.trace(R)
    
    if trace > 0:
        S = np.sqrt(trace + 1.0) * 2  # S = 4 * qw
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2  # S = 4 * qx
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2  # S = 4 * qy
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2  # S = 4 * qz
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S
    
    return np.array([qw, qx, qy, qz])


def quaternion_to_rotation_matrix(q):
    """
    Convert a quaternion to a 3x3 rotation matrix.
    q: [q1, q2, q3, q4]
    """
    q = q / np.linalg.norm(q)
    q1, q2, q3, q4 = q
    return np.array([
        [1 - 2*(q3**2 + q4**2), 2*(q2*q3 - q1*q4), 2*(q2*q4 + q1*q3)],
        [2*(q2*q3 + q1*q4), 1 - 2*(q2**2 + q4**2), 2*(q3*q4 - q1*q2)],
        [2*(q2*q4 - q1*q3), 2*(q3*q4 + q1*q2), 1 - 2*(q2**2 + q3**2)]
    ])


def plot_ellipsoid(ax, center=np.zeros(3), R=np.eye(3), radii=np.ones(3)):
    """
    Plots a unit sphere and its transformation into an ellipsoid.

    Parameters:
    - center: numpy.ndarray, the center of the ellipsoid.
    - R: numpy.ndarray, the rotation matrix for the ellipsoid.
    - radii: numpy.ndarray, the radii of the ellipsoid along its principal axes.
    """
    # Generate a unit sphere
    u = np.linspace(0, 2 * np.pi, 30)
    v = np.linspace(0, np.pi, 30)
    x = np.outer(np.cos(u), np.sin(v))
    y = np.outer(np.sin(u), np.sin(v))
    z = np.outer(np.ones_like(u), np.cos(v))

    # Transform the unit sphere to the ellipsoid
    ellipsoid = np.array([x.ravel(), y.ravel(), z.ravel()])
    ellipsoid = np.dot(R, ellipsoid * radii[:, None]) + center[:, None]
    x_ellipsoid, y_ellipsoid, z_ellipsoid = ellipsoid.reshape(3, x.shape[0], x.shape[1])

    ax.plot_surface(x_ellipsoid, y_ellipsoid, z_ellipsoid, color='orange', alpha=0.3, edgecolor='none')
    ax.set_title("Ellipsoid")
    set_axes_equal(ax)


import numpy as np

def construct_affine_matrix_with_two_rotations(center, evecs, radii, final_rot=None):
    """
    Construct a 4x4 affine transformation matrix corresponding to the transform function
    with two rotations, scaling, and translation.
    
    Parameters:
        center (numpy.ndarray): 1x3 array representing the center of the ellipsoid.
        evecs (numpy.ndarray): 3x3 matrix of eigenvectors (axes of the ellipsoid).
        radii (numpy.ndarray): 1x3 array of ellipsoid radii.
    
    Returns:
        numpy.ndarray: 4x4 affine transformation matrix.
    """
    # Create a 4x4 identity matrix
    affine_matrix = np.eye(4)

    # Scaling matrix (3x3)
    scaling_matrix = np.diag(1 / radii)

    # First rotation: evecs.T (align ellipsoid axes with coordinate axes)
    first_rotation = evecs.T

    # Second rotation: evecs (rotate back to original orientation)
    second_rotation = evecs

    # Combine scaling and first rotation
    rotation_scaling_matrix = np.dot(scaling_matrix, first_rotation)

    # Combine with second rotation
    full_linear_transform = np.dot(second_rotation, rotation_scaling_matrix)

    if final_rot is not None:
        full_linear_transform = np.dot(final_rot, full_linear_transform)

    # Insert the 3x3 linear transformation into the affine matrix
    affine_matrix[:3, :3] = full_linear_transform

    # Translation (negative center)
    affine_matrix[:3, 3] = -np.dot(full_linear_transform, center)

    return affine_matrix


if __name__ == "__main__":
    q = np.random.uniform(-1, 1, 4)
    q /= np.linalg.norm(q)
    R = quaternion_to_rotation_matrix(q)
    q2 = rotation_matrix_to_quaternion(R)
    assert (np.allclose(q, q2) or np.allclose(q, -q2))



