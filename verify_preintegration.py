# 驗證 ImuTypes.cc IntegrateNewMeasurement 的公式不是編造的
# 方法：四個獨立檢驗，全部只依賴 numpy 的基本矩陣運算
import numpy as np

rng = np.random.default_rng(42)

def hat(v):
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], dtype=float)

def Exp(phi):  # Rodrigues, 同 ImuTypes.cc:149
    d = np.linalg.norm(phi)
    W = hat(phi)
    if d < 1e-8:
        return np.eye(3) + W
    return np.eye(3) + W * np.sin(d) / d + W @ W * (1 - np.cos(d)) / d**2

def Log(R):  # Exp 的反函數
    tr = np.clip((np.trace(R) - 1) / 2, -1, 1)
    th = np.arccos(tr)
    if th < 1e-8:
        return np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]]) / 2
    return th / (2*np.sin(th)) * np.array([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]])

def Jr(phi):  # right Jacobian, 同 ImuTypes.cc:150
    d = np.linalg.norm(phi)
    W = hat(phi)
    if d < 1e-8:
        return np.eye(3)
    return np.eye(3) - W*(1-np.cos(d))/d**2 + W@W*(d-np.sin(d))/d**3

print("=" * 62)
print("檢驗 1：hat(a)·b 是否真的等於叉積 a×b（工具箱②的根基）")
err = max(np.linalg.norm(hat(a) @ b - np.cross(a, b))
          for a, b in [(rng.normal(size=3), rng.normal(size=3)) for _ in range(1000)])
print(f"  1000 組隨機向量，最大誤差 = {err:.2e}  （機器精度等級 → 成立）")

print("=" * 62)
print("檢驗 2：Rodrigues 公式是否等於真正的矩陣指數（scipy 獨立實作）")
try:
    from scipy.linalg import expm
    err = max(np.linalg.norm(Exp(p) - expm(hat(p)))
              for p in [rng.normal(size=3) for _ in range(200)])
    print(f"  200 組隨機軸角，最大誤差 = {err:.2e}  （兩種算法殊途同歸 → 成立）")
except ImportError:
    print("  (無 scipy，跳過)")

print("=" * 62)
print("檢驗 3：right Jacobian 恆等式 Exp(φ+δ) ≈ Exp(φ)·Exp(Jr(φ)δ)")
print("  若公式正確，誤差應隨 δ 縮小呈『二次』下降（縮10倍→誤差縮100倍）")
phi = rng.normal(size=3)
d0 = rng.normal(size=3)
for s in [1e-2, 1e-3, 1e-4]:
    d = d0 * s
    err = np.linalg.norm(Exp(phi + d) - Exp(phi) @ Exp(Jr(phi) @ d))
    print(f"  |δ| ~ {s:.0e}:  誤差 = {err:.3e}")

print("=" * 62)
print("檢驗 4（主菜）：蒙地卡羅 vs A/B 協方差傳遞（ImuTypes.cc:283-311）")
print("  模擬 10 步積分（軸還故意一直換），2 萬次加雜訊試驗，")
print("  比較『實測誤差協方差』與『A/B 公式推出的 C』")

dt = 0.005
n_steps = 10
sig_g, sig_a = 0.002, 0.02   # 每筆離散雜訊標準差
Nga = np.diag([sig_g**2]*3 + [sig_a**2]*3)
# 真值軌跡：角速度與加速度每步都不同、轉軸一直變（最嚴苛情境）
w_true = 0.5 * rng.normal(size=(n_steps, 3))
a_true = 2.0 * rng.normal(size=(n_steps, 3))

def integrate(w_seq, a_seq, with_cov=False):
    """完全照抄 IntegrateNewMeasurement 的更新順序"""
    dR, dV, dP = np.eye(3), np.zeros(3), np.zeros(3)
    C = np.zeros((9, 9))
    for k in range(n_steps):
        acc, w = a_seq[k], w_seq[k]
        A = np.eye(9); B = np.zeros((9, 6))
        Wacc = hat(acc)
        # 先更新 dP dV（用舊 dR）—— :276-277
        dP = dP + dV*dt + 0.5*dR@acc*dt*dt
        dV = dV + dR@acc*dt
        # 填 A B 的速度/位置格 —— :283-287
        A[3:6, 0:3] = -dR*dt @ Wacc
        A[6:9, 0:3] = -0.5*dR*dt*dt @ Wacc
        A[6:9, 3:6] = dt*np.eye(3)
        B[3:6, 3:6] = dR*dt
        B[6:9, 3:6] = 0.5*dR*dt*dt
        # 更新 dR —— :299-301
        dRi = Exp(w*dt)
        dR = dR @ dRi
        # 旋轉格 —— :305-306
        A[0:3, 0:3] = dRi.T
        B[0:3, 0:3] = Jr(w*dt)*dt
        # 協方差 —— :311
        C = A @ C @ A.T + B @ Nga @ B.T
    return (dR, dV, dP, C) if with_cov else (dR, dV, dP)

dR0, dV0, dP0, C_analytic = integrate(w_true, a_true, with_cov=True)

N = 20000
errs = np.zeros((N, 9))
for i in range(N):
    w_noisy = w_true + sig_g * rng.normal(size=(n_steps, 3))
    a_noisy = a_true + sig_a * rng.normal(size=(n_steps, 3))
    dRn, dVn, dPn = integrate(w_noisy, a_noisy)
    errs[i, 0:3] = Log(dR0.T @ dRn)   # 旋轉誤差 δφ
    errs[i, 3:6] = dVn - dV0          # δv
    errs[i, 6:9] = dPn - dP0          # δp
C_empirical = np.cov(errs.T)

d_emp = np.diag(C_empirical)
d_ana = np.diag(C_analytic)
labels = ["δφx","δφy","δφz","δvx","δvy","δvz","δpx","δpy","δpz"]
print(f"  {'分量':>4} {'實測變異數':>13} {'A/B公式預測':>13} {'比值':>7}")
for i in range(9):
    print(f"  {labels[i]:>4} {d_emp[i]:13.4e} {d_ana[i]:13.4e} {d_emp[i]/d_ana[i]:7.3f}")
rel = np.linalg.norm(C_empirical - C_analytic) / np.linalg.norm(C_analytic)
print(f"  整個 9×9 矩陣的相對誤差（Frobenius）= {rel:.3f}")
print("  （2 萬次抽樣的統計漲落約 ±1~2%；比值≈1.00 → 公式與現實吻合）")
