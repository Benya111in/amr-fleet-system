"""c04: smallest attainable two-sided exact Wilcoxon signed-rank p-value vs n (all differences same sign)."""
from scipy.stats import wilcoxon
import numpy as np
for n in (5, 6, 7, 8, 10, 30):
    d = np.arange(1, n+1, dtype=float)       # all positive, distinct
    p = wilcoxon(d, alternative="two-sided", method="exact").pvalue
    print("n=%2d  min two-sided exact p = %.3g  (2/2^n = %.3g)" % (n, p, 2/2**n))
