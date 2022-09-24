from exp_and_log_map import log_map_from_SO3_to_so3
import numpy as np


# dR(R1,R2) = ||log(R1_inv * R2)||_F / sqrt(2) -> argmin for R in SO3 sum(i=1 to n) {dR(R, Ri)^2}
def geodesic_metric(R1: np.ndarray, R2: np.ndarray) -> float:
    k, theta, R =  log_map_from_SO3_to_so3(np.linalg.inv(R1) @ R2)
    return np.linalg.norm(k)*np.sqrt(2)

def hyperbolic_metric(R1: np.ndarray, R2: np.ndarray) -> float:
    k1, theta1, R1 =  log_map_from_SO3_to_so3(R1)
    k2, theta2, R2 =  log_map_from_SO3_to_so3(R2)
    return np.linalg.norm(k2-k1)

def frobenius_metric(R1: np.ndarray, R2: np.ndarray):
    return np.linalg.norm(R1-R2)

def compute_metrics(R1: np.ndarray, Rs_interp: list):
    geodesic_metrics = []
    hyperbolic_metrics = []
    frobenius_metrics = []
    for i, R_interp in enumerate(Rs_interp):
        if i==0:
            continue
        geodesic_metrics.append(geodesic_metric(R1, R_interp))
        hyperbolic_metrics.append(hyperbolic_metric(R1, R_interp))
        frobenius_metrics.append(frobenius_metric(R1, R_interp))
    return geodesic_metrics, hyperbolic_metrics, frobenius_metrics


