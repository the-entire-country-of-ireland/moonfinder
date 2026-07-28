
import re
import numpy as np

def split_into_floats(input_string):
    # Regular expression to match floats (including integers)
    float_pattern = r'-?\d+\.?\d*'
    
    # Find all matches in the input string
    matches = re.findall(float_pattern, input_string)
    
    # Convert matches to floats
    floats = [float(match) for match in matches]
    
    return floats

from scipy.optimize import minimize
import numpy as np

# Function to represent the ellipse equation
def ellipse_residuals_l1(params, x, y):
    xc, yc, a, b, theta = params
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    x_rot = cos_theta * (x - xc) + sin_theta * (y - yc)
    y_rot = -sin_theta * (x - xc) + cos_theta * (y - yc)
    residuals = np.abs((x_rot / a) ** 2 + (y_rot / b) ** 2 - 1)  # L1 norm (absolute residuals)
    return residuals

# Fit the ellipse to the 2D data using L1 norm
def fit_ellipse_l1(data_2d):
    x = data_2d[:, 0]
    y = data_2d[:, 1]
    x_mean, y_mean = np.mean(x), np.mean(y)
    initial_guess = [x_mean, y_mean, np.std(x), np.std(y), 0]  # Initial guess for xc, yc, a, b, theta

    # Minimize the sum of absolute residuals
    result = minimize(
        lambda params: np.sum(ellipse_residuals_l1(params, x, y)),
        initial_guess,
        method='L-BFGS-B',
        bounds=[(None, None), (None, None), (1e-5, None), (1e-5, None), (None, None)]
    )
    return result.x  # Fitted parameters: xc, yc, a, b, theta

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import least_squares

# Function to represent the ellipse equation
def ellipse_residuals(params, x, y):
    xc, yc, a, b, theta = params
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    x_rot = cos_theta * (x - xc) + sin_theta * (y - yc)
    y_rot = -sin_theta * (x - xc) + cos_theta * (y - yc)
    return (x_rot / a) ** 2 + (y_rot / b) ** 2 - 1

# Fit the ellipse to the 2D data
def fit_ellipse(data_2d):
    x = data_2d[:, 0]
    y = data_2d[:, 1]
    x_mean, y_mean = np.mean(x), np.mean(y)
    initial_guess = [x_mean, y_mean, np.std(x), np.std(y), 0]  # Initial guess for xc, yc, a, b, theta
    result = least_squares(ellipse_residuals, initial_guess, args=(x, y))
    return result.x  # Fitted parameters: xc, yc, a, b, theta

# Generate points for the fitted ellipse
def generate_ellipse_points(params, num_points=100):
    xc, yc, a, b, theta = params
    t = np.linspace(0, 2 * np.pi, num_points)
    ellipse_x = a * np.cos(t)
    ellipse_y = b * np.sin(t)
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    x_rot = cos_theta * ellipse_x - sin_theta * ellipse_y + xc
    y_rot = sin_theta * ellipse_x + cos_theta * ellipse_y + yc
    return x_rot, y_rot

import numpy as np

def combined_affine_matrix(center_3d, pca_components, ellipse_params):
    """
    Create a 4x4 affine transformation matrix to map 3D data (centered and PCA-reduced)
    to a 2D unit circle.

    Parameters:
        center_3d: np.ndarray
            The 3D center of the data (shape: [3]).
        pca_components: np.ndarray
            The PCA components (shape: [2, 3]) used for 3D-to-2D reduction.
        ellipse_params: list or np.ndarray
            Ellipse parameters [xc, yc, a, b, theta], where:
            - xc, yc: Center of the 2D ellipse
            - a, b: Semi-major and semi-minor axes
            - theta: Rotation angle of the ellipse (in radians)

    Returns:
        np.ndarray: 4x4 affine transformation matrix.
    """
    # Unpack ellipse parameters
    xc, yc, a, b, theta = ellipse_params

    # Step 1: Translation matrix (to move the 3D center to the origin)
    translation_matrix_3d = np.array([
        [1, 0, 0, -center_3d[0]],
        [0, 1, 0, -center_3d[1]],
        [0, 0, 1, -center_3d[2]],
        [0, 0, 0, 1]
    ])

    # Step 2: PCA rotation matrix (to align 3D axes with PCA components)
    pca_rotation_matrix = np.eye(4)
    pca_rotation_matrix[:3, :3] = np.vstack([pca_components[0], pca_components[1], np.cross(pca_components[0], pca_components[1])])

    # Step 3: Translation matrix (to move the 2D ellipse center to the origin)
    translation_matrix_2d = np.array([
        [1, 0, 0, -xc],
        [0, 1, 0, -yc],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    # Step 4: Rotation matrix (to align the ellipse axes with the coordinate axes)
    cos_theta = np.cos(-theta)
    sin_theta = np.sin(-theta)
    rotation_matrix_2d = np.array([
        [cos_theta, -sin_theta, 0, 0],
        [sin_theta, cos_theta,  0, 0],
        [0,         0,          1, 0],
        [0,         0,          0, 1]
    ])

    # Step 5: Scaling matrix (to scale the ellipse to the unit circle)
    scaling_matrix_2d = np.array([
        [1/a, 0,   0, 0],
        [0,   1/b, 0, 0],
        [0,   0,   1, 0],
        [0,   0,   0, 1]
    ])

    # Combine transformations: Scaling * Rotation * Translation (2D) * PCA Rotation * Translation (3D)
    affine_matrix = scaling_matrix_2d @ rotation_matrix_2d @ translation_matrix_2d @ pca_rotation_matrix @ translation_matrix_3d

    return affine_matrix

def print_affine_matrix_formatted(matrix):
    """
    Print the 4x4 affine transformation matrix in the specified format.
    """
    print("// Affine transformation matrix (4x4)")
    print("const float AFFINE_TRANSFORMATION[4][4] = {")
    for row in matrix:
        formatted_row = ", ".join(f"{value:.6f}f" for value in row)
        print(f"    {{{formatted_row}}},")
    print("};")

import numpy as np

def apply_affine_transformation(matrix, vector):
    """
    Apply a 4x4 affine transformation matrix to a 3D vector.

    Parameters:
        matrix: np.ndarray
            The 4x4 affine transformation matrix.
        vector: np.ndarray
            The 3D vector to transform (shape: [3]).

    Returns:
        np.ndarray: The transformed 3D vector (shape: [3]).
    """
    # Convert the 3D vector to a 4D homogeneous vector
    homogeneous_vector = np.append(vector, 1.0)  # Add the homogeneous coordinate (w = 1)

    # Apply the affine transformation
    transformed_homogeneous = matrix @ homogeneous_vector

    # Convert back to a 3D vector by discarding the homogeneous coordinate
    transformed_vector = transformed_homogeneous[:3] / transformed_homogeneous[3]

    return transformed_vector


from sklearn.decomposition import PCA
import numpy as np
from scipy.optimize import minimize

def learnable_pca(data, n_components=2):
    """
    Perform PCA with a learnable center.

    Parameters:
        data: np.ndarray
            The input data (shape: [n_samples, n_features]).
        n_components: int
            Number of principal components to retain.

    Returns:
        center: np.ndarray
            The optimized center of the data.
        components: np.ndarray
            The principal components (shape: [n_components, n_features]).
    """
    n_samples, n_features = data.shape

    # Initialize the center as the mean of the data
    initial_center = np.mean(data, axis=0)

    # Define the loss function (reconstruction error)
    def loss_function(center):
        centered_data = data - center
        pca = PCA(n_components=n_components)
        pca.fit(centered_data)
        reconstruction = pca.inverse_transform(pca.transform(centered_data))
        return np.sum((centered_data - reconstruction) ** 2)

    # Optimize the center
    result = minimize(loss_function, initial_center, method='L-BFGS-B')
    optimized_center = result.x

    # Perform PCA on the centered data using the optimized center
    centered_data = data - optimized_center
    pca = PCA(n_components=n_components)
    pca.fit(centered_data)
    components = pca.components_

    return optimized_center, components


def process_file_to_raw_data(file_path, skip=10):
    A = []

    with open(file_path, "r") as f:
        data = f.readlines()
    if skip > 0:
        data = data[skip:-skip]  # remove header and footer
    data = list(set(data))  # get rid of duplicates

    for line in data:
        coords = split_into_floats(line)
        if max(coords) == 0 and min(coords) == 0:
            continue
        A.append(coords)
    return np.array(A)


def plot_ellipsoid(ax, params):
    """
    Plots the original 3D data and the fitted ellipsoid.

    Parameters:
    ax : mpl_toolkits.mplot3d.axes3d.Axes3D
        The 3D axis to plot on.
    params : numpy.ndarray
        Parameters of the fitted ellipsoid [xc, yc, zc, a, b, c, alpha, beta, gamma].
    """
    # Fitted ellipsoid
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x = params[0] + params[3] * np.outer(np.cos(u), np.sin(v))
    y = params[1] + params[4] * np.outer(np.sin(u), np.sin(v))
    z = params[2] + params[5] * np.outer(np.ones_like(u), np.cos(v))
    ax.plot_surface(x, y, z, color='pink', alpha=0.3)
    ax.set_title('Fitted Ellipsoid')

# Import necessary libraries
import numpy as np
from scipy.linalg import svd

# Function to fit a plane and find its normal vector
def fit_plane(points):
    centroid = np.mean(points, axis=0)
    centered_points = points - centroid
    _, _, vh = svd(centered_points)
    normal_vector = vh[-1]
    return -normal_vector, centroid

# Function to compute a rotation matrix to align a vector with the z-axis
def rotation_matrix_to_z(normal_vector):
    z_axis = np.array([0, 0, 1])
    v = np.cross(normal_vector, z_axis)
    c = np.dot(normal_vector, z_axis)
    s = np.linalg.norm(v)
    if s == 0:  # Already aligned
        return np.eye(3)
    vx = np.array([[0, -v[2], v[1]],
                   [v[2], 0, -v[0]],
                   [-v[1], v[0], 0]])
    R = np.eye(3) + vx + (vx @ vx) * ((1 - c) / (s ** 2))
    return R


# Function to generate a circle on the sphere at a given theta
def generate_circle_on_sphere(theta, num_points=100):
    phi = np.linspace(0, 2 * np.pi, num_points)  # Azimuthal angle
    x = np.sin(theta) * np.cos(phi)
    y = np.sin(theta) * np.sin(phi)
    z = np.cos(theta) * np.ones_like(phi)
    return np.vstack((x, y, z)).T




import numpy as np

def davenport_q_method(body, refv, wght):
    """
    Compute the optimal quaternion using Davenport's Q-Method.

    Args:
        body (np.ndarray): Nx3 array of body-frame vectors.
        refv (np.ndarray): Nx3 array of reference-frame vectors.
        wght (np.ndarray): N-element array of weights.

    Returns:
        np.ndarray: Optimal quaternion as a 4-element array [w, x, y, z].
    """
    # Ensure inputs are NumPy arrays
    body = np.asarray(body)
    refv = np.asarray(refv)
    wght = np.asarray(wght)

    # Build the 3x3 B matrix
    B = np.zeros((3, 3))
    for i in range(len(wght)):
        B += wght[i] * np.outer(body[i], refv[i])

    # Compute the symmetric 4x4 K matrix
    sigma = np.trace(B)
    S = B + B.T
    Z = np.array([
        B[1, 2] - B[2, 1],
        B[2, 0] - B[0, 2],
        B[0, 1] - B[1, 0]
    ])
    K = np.zeros((4, 4))
    K[0, 0] = sigma
    K[0, 1:4] = Z
    K[1:4, 0] = Z
    K[1:4, 1:4] = S - sigma * np.eye(3)

    # Power iteration to find the dominant eigenvector (quaternion)
    q = np.array([1, 0, 0, 0])  # Initial guess for quaternion
    for _ in range(100):
        q_next = K @ q
        q = q_next / np.linalg.norm(q)

    return q