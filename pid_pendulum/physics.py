
    p = linalg.solve_continuous_are(A, B, Q, R)
    k = np.linalg.inv(R) @ B.T @ p
    return k
