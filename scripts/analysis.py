
import re
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import cyipopt

def serialize(A_hat, b_hat):
    try:
        with open("D:/data/calibration.txt", 'w') as f:
            for row in A_hat:
                f.write(' '.join(map(str, row)) + '\n')
            f.write(' '.join(map(str, b_hat)) + '\n')
    except:
        pass
    finally:
        for row in A_hat:
            print(', '.join(map(str, row)) + ',' )
        print(', '.join(map(str, b_hat)) )


def normalize(vec):
    if len(vec.shape) == 1:
        return vec / np.linalg.norm(vec)
    return vec / np.linalg.norm(vec, axis=1, keepdims=True)


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

def load_data(filename="magacc_readings_8.txt", atol=0.4):
    data = process_file_to_raw_data(filename)
    mag = data[:,:3]
    acc = data[:,3:]

    norm = np.linalg.norm(acc, 2, axis=1)
    idcs = np.isclose(norm, 9.8, atol=atol)

    mag = mag[idcs]
    acc = acc[idcs]

    mag = np.array(mag)
    acc = np.array(acc)
    return mag, acc

def split_into_floats(input_string):
    # Regular expression to match floats (including integers)
    float_pattern = r'-?\d+\.?\d*'
    
    # Find all matches in the input string
    matches = re.findall(float_pattern, input_string)
    
    # Convert matches to floats
    floats = [float(match) for match in matches]
    
    return floats

def grade(a, b):
    z = np.array([i@j for i,j in zip(a,b)])
    return np.mean(z), np.std(z)



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




# Symmetric 4x4 parameter ordering:
# (00, 11, 22, 33, 01, 02, 03, 12, 13, 23)
Q_PAIRS = (
    (0, 0), (1, 1), (2, 2), (3, 3),
    (0, 1), (0, 2), (0, 3),
    (1, 2), (1, 3), (2, 3),
)


def q_to_matrix(q):
    Q = np.zeros((4, 4))
    for value, (i, j) in zip(q, Q_PAIRS):
        Q[i, j] = Q[j, i] = value
    return Q


def matrix_to_q(Q):
    return np.array([Q[i, j] for i, j in Q_PAIRS])


def quadratic_features(x):
    """phi(x)^T q = x^T Q x."""
    return np.array([
        x[0] ** 2,
        x[1] ** 2,
        x[2] ** 2,
        x[3] ** 2,
        2 * x[0] * x[1],
        2 * x[0] * x[2],
        2 * x[0] * x[3],
        2 * x[1] * x[2],
        2 * x[1] * x[3],
        2 * x[2] * x[3],
    ])


class StreamingExactCalibration:
    """
    Decision variables:
        W: 3x4 affine calibration matrix
        Q: symmetric 4x4 matrix

    Objective:
        alpha/2 * mean[(a^T W x - T)^2]
      + beta /2 * mean[(x^T Q x - 1)^2]

    Exact constraints:
        Q = W^T W
    """

    def __init__(self, target_dot, alpha=1.0, beta=1.0):
        self.T = float(target_dot)
        self.alpha = float(alpha)
        self.beta = float(beta)

        self.Ha = np.zeros((12, 12))
        self.ga = np.zeros(12)

        self.Hn = np.zeros((10, 10))
        self.gn = np.zeros(10)

        self.n = 0

        # Lower-triangular indices required by Ipopt.
        self.hess_rows, self.hess_cols = np.tril_indices(22)

        # Constant Hessian of each equality constraint.
        self.constraint_hessians = self._make_constraint_hessians()

    def update(self, mag_raw, acc, weight=1.0, W0=None):
        mag_raw = np.asarray(mag_raw, dtype=float)
        acc = np.asarray(acc, dtype=float)

        acc = acc / np.linalg.norm(acc)
        x = np.r_[mag_raw, 1.0]

        # Compute scale factor based on previous solution
        scale = 1.0 if W0 is None else np.linalg.norm(W0 @ x)

        # a^T W x = z^T vec_row(W)
        z = np.kron(acc, x)

        # x^T Q x = phi^T q
        phi = quadratic_features(x)

        self.Ha += weight * np.outer(z, z)
        self.ga += weight * self.T * z * scale

        self.Hn += weight * np.outer(phi, phi)
        self.gn += weight * phi

        self.n += 1

    def _split(self, theta):
        W = theta[:12].reshape(3, 4)
        q = theta[12:]
        return W, q

    def objective(self, theta):
        w = theta[:12]
        q = theta[12:]
        scale = 1.0 / max(self.n, 1)

        return 0.5 * scale * (
            self.alpha * (w @ self.Ha @ w - 2 * self.ga @ w)
            + self.beta * (q @ self.Hn @ q - 2 * self.gn @ q)
        )

    def gradient(self, theta):
        w = theta[:12]
        q = theta[12:]
        scale = 1.0 / max(self.n, 1)

        return scale * np.r_[
            self.alpha * (self.Ha @ w - self.ga),
            self.beta * (self.Hn @ q - self.gn),
        ]

    def constraints(self, theta):
        W, q = self._split(theta)
        gram = W.T @ W

        return np.array([
            q[k] - gram[i, j]
            for k, (i, j) in enumerate(Q_PAIRS)
        ])

    def jacobian(self, theta):
        W, _ = self._split(theta)

        # Dense 10x22 Jacobian.
        J = np.zeros((10, 22))

        for k, (p, q) in enumerate(Q_PAIRS):
            # Derivative with respect to q_k.
            J[k, 12 + k] = 1.0

            # Derivative of -(W^T W)_{pq}.
            for row in range(3):
                wp = 4 * row + p
                wq = 4 * row + q

                if p == q:
                    J[k, wp] = -2.0 * W[row, p]
                else:
                    J[k, wp] = -W[row, q]
                    J[k, wq] = -W[row, p]

        return J.ravel(order="C")

    def _make_constraint_hessians(self):
        """
        Hessians of:
            c_pq = Q_pq - sum_r W[r,p] W[r,q].

        Only the W-W blocks are nonzero.
        """
        hessians = np.zeros((10, 22, 22))

        for k, (p, q) in enumerate(Q_PAIRS):
            for row in range(3):
                wp = 4 * row + p
                wq = 4 * row + q

                if p == q:
                    hessians[k, wp, wp] = -2.0
                else:
                    hessians[k, wp, wq] = -1.0
                    hessians[k, wq, wp] = -1.0

        return hessians

    def hessianstructure(self):
        return self.hess_rows, self.hess_cols

    def hessian(self, theta, lagrange, obj_factor):
        scale = 1.0 / max(self.n, 1)

        H = np.zeros((22, 22))

        # Objective Hessian.
        H[:12, :12] = obj_factor * scale * self.alpha * self.Ha
        H[12:, 12:] = obj_factor * scale * self.beta * self.Hn

        # Equality-constraint Hessians.
        H += np.tensordot(
            lagrange,
            self.constraint_hessians,
            axes=(0, 0),
        )

        return H[self.hess_rows, self.hess_cols]

    def solve(self, W0=None):
        if self.n == 0:
            raise ValueError("No samples have been accumulated.")

        if W0 is None:
            # Alignment-only least-squares initialization.
            ridge = 1e-8 * np.eye(12)
            w0 = np.linalg.solve(self.Ha + ridge, self.ga)
            W0 = w0.reshape(3, 4)

        # Start exactly feasible: Q0 = W0^T W0.
        Q0 = W0.T @ W0
        theta0 = np.r_[W0.ravel(), matrix_to_q(Q0)]

        nlp = cyipopt.Problem(
            n=22,
            m=10,
            problem_obj=self,
            lb=np.full(22, -np.inf),
            ub=np.full(22, np.inf),
            cl=np.zeros(10),
            cu=np.zeros(10),
        )

        nlp.add_option("tol", 1e-9)
        nlp.add_option("constr_viol_tol", 1e-10)
        nlp.add_option("max_iter", 500)
        nlp.add_option("mu_strategy", "adaptive")
        nlp.add_option("hessian_approximation", "exact")
        nlp.add_option("print_level", 0)

        theta, info = nlp.solve(theta0)

        W, q = self._split(theta)
        Q = q_to_matrix(q)

        return W, Q, info
    