"""run_stanley_eval.m 결과(stanley_eval.mat)로 보고서 그림 3장 + metrics_simulink.json 생성.
사용: python plot_simulink.py <stanley_eval.mat> [출력 폴더]
"""
import sys, os, json
import numpy as np
from scipy.io import loadmat
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams.update({"font.family": "AppleGothic", "axes.unicode_minus": False, "font.size": 9})
src = sys.argv[1]
out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(os.path.dirname(__file__), "..", "results")
os.makedirs(out, exist_ok=True)
m = loadmat(src, squeeze_me=True, struct_as_record=False)
L = m["logs"]
lg = lambda k: getattr(L, k)

# 그림 1: 경로 4종 궤적 (Stanley, kinematic)
fig, axs = plt.subplots(1, 4, figsize=(11, 2.9))
titles = ["직선", "원형(R=100 m)", "우회전 폐곡선", "레이스트랙"]
for ax, name, ttl in zip(axs, ["linear", "circular", "right_turn", "racetrack"], titles):
    o = lg(f"{name}_m1_stanley")
    ax.plot(o.Path[0], o.Path[1], "--", color="0.6", lw=1.5, label="목표 경로")
    ax.plot(o.X, o.Y, "b", lw=1, label="Stanley 궤적")
    ax.set_title(ttl); ax.set_aspect("equal", "datalim"); ax.grid(alpha=.3); ax.set_xlabel("X [m]")
axs[0].set_ylabel("Y [m]"); axs[0].legend(fontsize=7, loc="upper left")
fig.tight_layout(); fig.savefig(os.path.join(out, "fig1_trajectories.png"), dpi=200); plt.close(fig)

# 그림 2: 직선 초기오차 1 m 응답
fig, axs = plt.subplots(1, 3, figsize=(11, 2.8))
for k in (5, 10, 20, 30):
    o = lg(f"step_k{k:02d}")
    axs[0].plot(o.t, o.e_y, lw=1.2, label=f"k={k/10:g}")
    axs[1].plot(o.t, np.degrees(o.SWA_act), lw=1.2)
o = lg("step_k40")
axs[2].plot(o.t, o.e_y, "r", lw=1)
axs[0].set(xlabel="시간 [s]", ylabel="e_y [m]", title="(a) 횡방향 오차, k ≤ 3", xlim=(0, 6))
axs[1].set(xlabel="시간 [s]", ylabel="SWA [deg]", title="(b) 실제 조향휠 각, k ≤ 3", xlim=(0, 6))
axs[2].set(xlabel="시간 [s]", ylabel="e_y [m]", title="(c) k = 4 : 발산")
axs[0].legend(fontsize=7)
for a in axs: a.grid(alpha=.3)
fig.tight_layout(); fig.savefig(os.path.join(out, "fig2_step_gain.png"), dpi=200); plt.close(fig)

# 그림 3: 레이스트랙 50 km/h — (a) PD vs Stanley (dynamic), (b) Stanley kinematic vs dynamic 확대
fig, axs = plt.subplots(2, 1, figsize=(9, 3.6), sharex=True)
pd_, sd, sk = lg("racetrack_m2_pd"), lg("racetrack_m2_stanley"), lg("racetrack_m1_stanley")
axs[0].plot(pd_.t, pd_.e_y, "r--", lw=1, label="PD (배포 기본)")
axs[0].plot(sd.t, sd.e_y, "b", lw=1, label="Stanley (k=2)")
axs[1].plot(sk.t, sk.e_y, color="0.5", lw=1, label="kinematic 모델")
axs[1].plot(sd.t, sd.e_y, "b", lw=1.2, label="dynamic 모델")
axs[1].set_ylim(-0.5, 0.5)
axs[0].set_ylabel("e_y [m]"); axs[1].set_ylabel("e_y [m]"); axs[1].set_xlabel("시간 [s]")
axs[0].set_title("(a) PD와 Stanley (dynamic 모델)", fontsize=9)
axs[1].set_title("(b) Stanley, 차량 모델별 (세로축 확대)", fontsize=9)
for a in axs: a.grid(alpha=.3); a.legend(fontsize=7, ncol=2, loc="upper left")
fig.tight_layout(); fig.savefig(os.path.join(out, "fig3_racetrack.png"), dpi=200); plt.close(fig)

print("figures saved to", out)
