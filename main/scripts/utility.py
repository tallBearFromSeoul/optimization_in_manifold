import numpy as np

def deg2rad(deg: float):
    return deg*np.pi/180.0
def rad2deg(rad: float):
    return rad*180.0/np.pi

def cross_product(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    if not isinstance(a, np.ndarray) or not isinstance(b, np.ndarray):
        raise NotNumpyArray
    if a.ndim != 1 or b.ndim != 1:
        raise NotOneDimArray
    if a.shape[0] != 3 or b.shape[0] != 3:
        raise Not3Vector
    return np.array([a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0]])

def trace(A: np.ndarray) -> float:
    if not isinstance(A, np.ndarray):
        raise NotNumpyArray
    if A.shape[0] != A.shape[1]:
        raise NonSquareMatrix
    trace = 0
    for i in range(A.shape[0]):
        trace += A[i][i]
    return trace

def vee_operator(A: np.ndarray) -> np.ndarray:
    if not isinstance(A, np.ndarray):
        raise NotNumpyArray
    if A.shape[0] != A.shape[1]:
        raise NonSquareMatrix
    if (-A != A.T).all():
        raise NonSymmetricMatrix
    return np.array([-A[1,2], A[0,2], -A[0,1]])

def hat_operator(v: np.ndarray) -> np.ndarray:
    return skew_symmetric(v)

# also known as hat operator
def skew_symmetric(v: np.ndarray) -> np.ndarray:
    if not isinstance(v, np.ndarray):
        raise NotNumpyArray
    if v.ndim != 1:
        raise NotOneDimArray
    return np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])

def quaternion_mult(q1: np.ndarray, q2: np.ndarray):
    if not isinstance(q1, np.ndarray) or not isinstance(q2, np.ndarray):
        raise NotNumpyArray
    if q1.ndim != 1 or q2.ndim != 1:
        raise NotOneDimArray
    if q1.shape[0] != 4 or q2.shape[0] != 4:
        raise Not4Vector
    q1r = q1[0]
    q1v = q1[1:]
    q2r = q2[0]
    q2v = q2[1:]
    qr = np.array([q1r*q2r - q1v.dot(q2v)])
    qv = q1r*q2v + q2r*q1v + cross_product(q1v, q2v)
    return np.hstack((qr, qv))

def quaternion_inverse(q: np.ndarray):
    if not isinstance(q, np.ndarray):
        raise NotNumpyArray
    if q.ndim != 1: 
        raise NotOneDimArray
    if q.shape[0] != 4:
        raise Not4Vector
    qr = q[0]
    qv = q[1:]
    q_norm = q.dot(q)
    q_inv = np.hstack((qr, -qv))/q_norm
    return q_inv

# q = [a, b, c, d].T = [qr, qv].T
# qr = a, qv = [b, c, d].T
# q = [cos(theta), w*sin(theta)].T
# where,
# theta = arccos(qr)
# w = qv / norm(qv)
# then,
# q^n = [cos(n*theta), w*sin(n*theta)].T
def quaternion_power(q: np.ndarray, n: float):
    if not isinstance(q, np.ndarray):
        raise NotNumpyArray
    if q.ndim != 1: 
        raise NotOneDimArray
    if q.shape[0] != 4:
        raise Not4Vector
    if not isinstance(n, float):
        raise NotFloat
    qr = q[0]
    qv = q[1:]
    qv_norm = np.linalg.norm(qv)
    theta = np.arccos(qr)
    w = qv/qv_norm
    return np.hstack((np.cos(n*theta), w*np.sin(n*theta)))


