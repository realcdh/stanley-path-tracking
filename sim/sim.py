"""PathTracking Simulink 모델(1차 배포본)의 Python 재현 + Stanley/PD 제어기.

배포 모델의 블록 순서를 그대로 따름 (Ts=0.01 s, 고정 스텝, 전진 오일러 이산 적분):
  Error Calculator -> Controller -> SWA 액추에이터(Unit Delay, 1/(0.1s+1), 포화 540 deg, 500 deg/s)
  -> /GR -> 차량 모델(kinematic 또는 dynamic bicycle) -> 전역 위치 적분
속도 제어는 배포본 기본값(Manual Switch=0, Ax_cmd=0)대로 50 km/h 등속, 노이즈 0.

사용: python sim.py <path_data_*.mat 이 있는 폴더> [출력 폴더]
"""
import sys, os, json
import numpy as np
from scipy.io import loadmat

TS, SIM_TIME = 0.01, 50.0
GR, L, ZETA_F = 15.0, 2.7, 0.6
LF, LR = L * (1 - ZETA_F), L * ZETA_F
M, IZ, CF, CR = 1573.0, 2873.0, 8.0e4 * 2, 8.0e4 * 2
VX0 = 50 / 3.6
PATHS = {1: "linear", 2: "circular", 3: "right_turn", 4: "racetrack"}


# ---------------------------------------------------------------- Error Calculator (배포 코드 이식)
class ErrorCalc:
    SRCH = 10

    def __init__(self, path):
        self.p = path
        self.N = path.shape[1]
        ds = np.hypot(*(path[:, 1] - path[:, 0]))
        self.closed = np.hypot(*(path[:, -1] - path[:, 0])) < 2 * ds
        self.idx = 1  # MATLAB 1-based, Unit Delay 초기값 1

    def wrap(self, i):
        if self.closed:
            return (i - 1) % self.N + 1
        off = 0
        if i.min() < 1:
            off = 1 - i.min()
        elif i.max() > self.N:
            off = self.N - i.max()
        return np.clip(i + off, 1, self.N)

    def __call__(self, X, Y, yaw):
        yaw = np.arctan2(np.sin(yaw), np.cos(yaw))
        iW = self.wrap(np.arange(self.idx - self.SRCH, self.idx + self.SRCH + 1))
        near = self.p[:, iW - 1]
        k = np.argmin((near[0] - X) ** 2 + (near[1] - Y) ** 2)
        self.idx = int(iW[k])

        t_span = np.linspace(0, 1, iW.size)
        px, py = np.polyfit(t_span, near[0], 2), np.polyfit(t_span, near[1], 2)
        dpx, dpy = np.array([2 * px[0], px[1]]), np.array([2 * py[0], py[1]])
        d2x, d2y = 2 * px[0], 2 * py[0]
        tq = np.linspace(0, 1, 101)
        t = tq[np.argmin((np.polyval(px, tq) - X) ** 2 + (np.polyval(py, tq) - Y) ** 2)]
        for _ in range(3):
            xt, yt = np.polyval(px, t), np.polyval(py, t)
            dxt, dyt = np.polyval(dpx, t), np.polyval(dpy, t)
            f = (xt - X) * dxt + (yt - Y) * dyt
            fp = dxt * dxt + dyt * dyt + (xt - X) * d2x + (yt - Y) * d2y
            if abs(fp) < 1e-12:
                break
            t = min(max(t - f / fp, 0), 1)
        xr, yr = np.polyval(px, t), np.polyval(py, t)
        dxr, dyr = np.polyval(dpx, t), np.polyval(dpy, t)
        if dxr * (near[0, -1] - near[0, 0]) + dyr * (near[1, -1] - near[1, 0]) < 0:
            dxr, dyr = -dxr, -dyr
        spd2 = max(dxr * dxr + dyr * dyr, 1e-9)
        yaw_road = np.arctan2(dyr, dxr)
        e_y = -np.sin(yaw_road) * (X - xr) + np.cos(yaw_road) * (Y - yr)
        e_yaw = np.arctan2(np.sin(yaw - yaw_road), np.cos(yaw - yaw_road))
        curv = np.clip((dxr * d2y - dyr * d2x) / spd2 ** 1.5, -1, 1)
        end = (not self.closed) and self.idx >= self.N - 1
        return e_y, e_yaw, curv, end


# ---------------------------------------------------------------- 제어기
def stanley(e_y, e_yaw, vx, k=2.0, eps=1.0):
    e_fa = e_y + LF * np.sin(e_yaw)
    return -(e_yaw + np.arctan2(k * e_fa, vx + eps))


def pd(e_y, e_yaw, vx, z=1.0, wn=1.0):  # 배포본 기본 제어기 (PID Control 블록의 PD 경로)
    k1, k2 = wn**2 * L / VX0**2, 2 * z * wn * L / VX0
    return -(k1 * e_y + k2 * e_yaw)


# ---------------------------------------------------------------- 시뮬레이션
def simulate(path, ctrl, model="kin", ey0=1.0, ephi0=0.0, rate_dps=500.0):
    ec = ErrorCalc(path)
    X, Y, yaw, vx = path[0, 0], path[1, 0] + ey0, ephi0, VX0
    beta = r = 0.0                       # dynamic model 상태
    swa_cmd_prev = swa_tf = swa_act = 0.0
    a_tf = np.exp(-TS / 0.1)
    rate = np.deg2rad(rate_dps) * TS
    log = {k: [] for k in ("t", "X", "Y", "e_y", "e_yaw", "swa_cmd", "swa_act", "ay", "curv")}
    for n in range(int(SIM_TIME / TS) + 1):
        e_y, e_yaw, curv, end = ec(X, Y, yaw)
        swa_cmd = ctrl(e_y, e_yaw, vx) * GR
        # 액추에이터: Unit Delay -> 1차 지연 (ZOH 이산화) -> 포화 -> 레이트 리미터
        sat = np.clip(swa_tf, -np.deg2rad(540), np.deg2rad(540))
        swa_act = swa_act + np.clip(sat - swa_act, -rate, rate) if n else 0.0
        delta = swa_act / GR
        if model == "kin":
            td = np.tan(delta)
            beta, r = np.arctan(LR * td / L), vx * td / L
            ay = vx * r
            beta_dot = r_dot = 0.0
        else:
            vxe = max(0.1, vx)
            af = beta + LF * r / vxe - delta
            ar = beta - LR * r / vxe
            ay = (-CF * af - CR * ar) / M
            beta_dot = -(CF + CR) / (M * vxe) * beta + (-(LF * CF - LR * CR) / (M * vxe**2) - 1) * r + CF / (M * vxe) * delta
            r_dot = (LR * CR - LF * CF) / IZ * beta - (LF**2 * CF + LR**2 * CR) / (IZ * vxe) * r + LF * CF / IZ * delta
        for k, v in zip(log, (n * TS, X, Y, e_y, e_yaw, swa_cmd, swa_act, ay, curv)):
            log[k].append(v)
        if end:
            break
        vy = vx * np.tan(beta)
        X += TS * (vx * np.cos(yaw) - vy * np.sin(yaw))
        Y += TS * (vx * np.sin(yaw) + vy * np.cos(yaw))
        yaw += TS * r
        if model == "dyn":
            beta += TS * beta_dot
            r += TS * r_dot
        swa_tf = a_tf * swa_tf + (1 - a_tf) * swa_cmd_prev
        swa_cmd_prev = swa_cmd
    return {k: np.array(v) for k, v in log.items()}


def metrics(lg):
    rmse = lambda x: float(np.sqrt(np.mean(np.square(x))))
    jy = np.diff(lg["ay"], prepend=lg["ay"][0]) / TS
    dswa = np.diff(lg["swa_act"], prepend=lg["swa_act"][0]) / TS
    return {
        "rmse_ey": rmse(lg["e_y"]), "max_ey": float(np.max(np.abs(lg["e_y"]))),
        "rmse_eyaw_deg": np.degrees(rmse(lg["e_yaw"])), "max_eyaw_deg": float(np.degrees(np.max(np.abs(lg["e_yaw"])))),
        "rmse_ay": rmse(lg["ay"]), "max_ay": float(np.max(np.abs(lg["ay"]))),
        "rmse_jerk_y": rmse(jy), "rmse_dswa": rmse(dswa),
        "ey_last5s_rmse": rmse(lg["e_y"][lg["t"] >= lg["t"][-1] - 5]),
    }


if __name__ == "__main__":
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": "AppleGothic", "axes.unicode_minus": False, "font.size": 9})

    data_dir = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(os.path.dirname(__file__), "..", "results")
    os.makedirs(out, exist_ok=True)
    paths = {m: loadmat(os.path.join(data_dir, f"path_data_{n}.mat"))["path"] for m, n in PATHS.items()}
    res = {}

    # 1) 경로 4종 x 차량모델 2종 : Stanley(k=기본) vs PD(배포 기본)
    K0 = 2.0
    for mode, p in paths.items():
        for mdl in ("kin", "dyn"):
            for name, c in (("stanley", lambda a, b, v: stanley(a, b, v, k=K0)), ("pd", pd)):
                lg = simulate(p, c, mdl)
                res[f"{PATHS[mode]}/{mdl}/{name}"] = metrics(lg)
                if mdl == "kin":
                    np.savez(os.path.join(out, f"log_{PATHS[mode]}_{name}.npz"), **lg)

    # 2) 게인 스윕 (racetrack, kinematic / dynamic)
    sweep = {}
    for mdl in ("kin", "dyn"):
        for k in (0.5, 1.0, 2.0, 4.0, 8.0):
            lg = simulate(paths[4], lambda a, b, v: stanley(a, b, v, k=k), mdl)
            sweep[f"{mdl}/k={k}"] = metrics(lg)
    res["sweep_racetrack"] = sweep

    # 3) 직선 초기오차 1 m 응답, 게인별
    step = {}
    for k in (0.5, 1.0, 2.0, 3.0, 4.0):
        step[k] = simulate(paths[1], lambda a, b, v: stanley(a, b, v, k=k), "kin")
    with open(os.path.join(out, "metrics.json"), "w") as f:
        json.dump(res, f, indent=1)

    # ---- 그림 1: 경로 4종 궤적 (Stanley, kinematic)
    fig, axs = plt.subplots(1, 4, figsize=(11, 2.9))
    for ax, (mode, p) in zip(axs, paths.items()):
        lg = np.load(os.path.join(out, f"log_{PATHS[mode]}_stanley.npz"))
        ax.plot(p[0], p[1], "--", color="0.6", lw=1.5, label="목표 경로")
        ax.plot(lg["X"], lg["Y"], "b", lw=1, label="Stanley 궤적")
        ax.set_title(["직선", "원형(R=100 m)", "우회전 폐곡선", "레이스트랙"][mode - 1])
        ax.set_aspect("equal", "datalim"); ax.grid(alpha=.3); ax.set_xlabel("X [m]")
    axs[0].set_ylabel("Y [m]"); axs[0].legend(fontsize=7, loc="upper left")
    fig.tight_layout(); fig.savefig(os.path.join(out, "fig1_trajectories.png"), dpi=200); plt.close(fig)

    # ---- 그림 2: 직선 초기오차 응답 (안정 게인 / 발산 게인)
    fig, axs = plt.subplots(1, 3, figsize=(11, 2.8))
    for k, lg in step.items():
        if k < 4:
            axs[0].plot(lg["t"], lg["e_y"], lw=1.2, label=f"k={k}")
            axs[1].plot(lg["t"], np.degrees(lg["swa_act"]), lw=1.2)
    lg = step[4.0]
    axs[2].plot(lg["t"], lg["e_y"], "r", lw=1)
    axs[0].set(xlabel="시간 [s]", ylabel="e_y [m]", title="(a) 횡방향 오차, k ≤ 3", xlim=(0, 6))
    axs[1].set(xlabel="시간 [s]", ylabel="SWA [deg]", title="(b) 실제 조향휠 각, k ≤ 3", xlim=(0, 6))
    axs[2].set(xlabel="시간 [s]", ylabel="e_y [m]", title="(c) k = 4 : 발산")
    axs[0].legend(fontsize=7)
    for a in axs: a.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(os.path.join(out, "fig2_step_gain.png"), dpi=200); plt.close(fig)

    # ---- 그림 3: 레이스트랙 시계열 (Stanley vs PD, kinematic)
    fig, axs = plt.subplots(3, 1, figsize=(9, 4.6), sharex=True)
    for name, st in (("stanley", "b"), ("pd", "r--")):
        lg = np.load(os.path.join(out, f"log_racetrack_{name}.npz"))
        lab = f"Stanley (k={K0})" if name == "stanley" else "PD (배포 기본)"
        axs[0].plot(lg["t"], lg["e_y"], st, lw=1, label=lab)
        axs[1].plot(lg["t"], np.degrees(lg["e_yaw"]), st, lw=1)
        axs[2].plot(lg["t"], lg["curv"], st if name == "stanley" else "none", lw=1)
    axs[0].set_ylabel("e_y [m]"); axs[1].set_ylabel("e_ψ [deg]"); axs[2].set_ylabel("곡률 [1/m]")
    axs[2].set_xlabel("시간 [s]"); axs[0].legend(fontsize=7, ncol=2)
    for a in axs: a.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(os.path.join(out, "fig3_racetrack.png"), dpi=200); plt.close(fig)

    print(json.dumps(res, indent=1))
