# Stanley 경로 추종 제어기 (PBL)

Simulink `PathTracking` 모델의 Path Tracking Controller 빈 슬롯(Multiport Switch 2번 데이터 포트)에 넣는 Stanley 제어기와, 결과 분석용 Python 재현 시뮬레이터.

배포 모델·경로 데이터(`.slx`, `path_data_*.mat`)는 수업 자료라 포함하지 않음.

## 제어 법칙

```
e_fa  = e_y + Lf·sin(e_ψ)                                   % 무게중심 오차 -> 전륜축 오차
δ     = -( e_ψ + atan( k·e_fa / (v_x + ε) ) ) + ff·δ_ff
δ_ff  = κ·( Lf + Kv·v_x² ),  Kv = m·Lr / (L·Cf)              % 곡률 피드포워드 (ff_st = 1)
```
부호: 모델 규약상 `e_y > 0`(차량이 경로 좌측), `e_ψ > 0`(좌측을 향함)일 때 우조향(δ < 0).

순수 Stanley의 원선회 정상상태 오차는 `e_fa = -((v+ε)/k)·κ·(Lf + Kv·v²)` (2DOF bicycle 선형화). Simulink 원형 경로 결과와 3~11% 이내로 일치.
`δ_ff`는 이 오차를 미리 보상하는 항. dynamic 모델 기준 계수라 kinematic 모델에서는 과보상 → `ff_st` 기본값 0, dynamic 평가 때 1.

## MATLAB 적용 (R2026a에서 검증)

**주의: MATLAB 설치 경로·현재 폴더·MATLAB 경로에 한글 폴더가 있으면 MATLAB Function 블록이 컴파일되지 않음**
(`atan2` 한 줄짜리 블록도 `Error in port widths or dimensions` / `Invalid character code sequence detected`).
배포 폴더를 영문 경로(예: `~/Documents/MATLAB/PBL_run`)에 복사해서 실행.

```matlab
addpath('<이 리포>/matlab')
add_stanley_block('<배포 폴더>')   % 1회. 원본은 *_backup.slx 로 백업
run_simulation                     % Controller_mode = 2 (Stanley)
run_stanley_eval                   % 경로 4종 x 차량모델 2종 x 제어기 2종 + 게인 스윕 → stanley_eval.mat
```
`add_stanley_block`: MATLAB Function 블록 추가·연결, `Controller` 상수를 `Controller_mode`로 변경, `init_param.m` 끝에 `stanley_param;` 추가.

모델 `InitFcn` 콜백이 시뮬레이션마다 `init_param`을 다시 부르므로, `stanley_param.m`은 이미 있는 `k_st`·`eps_st`·`Controller_mode`를 덮어쓰지 않음. 게인은 `stanley_param.m` 기본값을 고치거나, `clear all` 없이 작업공간에서 바꾼 뒤 실행.

## 팀원용: 시뮬레이션 돌리기 전 체크

1. **경로에 한글 금지.** MATLAB 설치 폴더, 실습 폴더, MATLAB 경로(addpath) 어디에든 한글이 있으면 MATLAB Function 블록 컴파일 실패.
   - MATLAB에서 `matlabroot` 확인 → 한글 있으면 앱을 영문 폴더로 이동
   - 실습 폴더를 `~/Documents/MATLAB/PBL_run` 같은 영문 경로에 복사해서 실행
2. **파라미터는 파일에서 바꾸기.** 모델 InitFcn이 시뮬레이션마다 `init_param.m`을 다시 불러 작업공간 값을 덮어씀. 게인은 `init_param.m`(또는 개인 param 파일)을 수정.
3. **제어기 추가 위치.** `Path Tracking Controller` 안 `Multiport Switch` 데이터 포트. 입력 수를 늘리고 `Controller` 상수 값으로 선택.
   - 제안 번호: 1 PID · 2 Stanley · 3 Pure Pursuit · 4 LQR
   - 같은 서브시스템에 Goto 태그 `e_y`, `e_yaw`, `vx`, `vy`, `yawrate`, `curv`, `Path_local` 있음 → From 블록으로 받으면 됨
   - 출력은 전륜 조향각 [rad] (좌회전 +). 뒤에서 GearRatio ×15가 곱해짐
   - 부호: `e_y > 0` 차량이 경로 왼쪽, `e_yaw > 0` 차량이 경로보다 왼쪽을 향함

## 결과 (Simulink, 50 km/h, 초기 횡오차 1 m)

횡오차 RMSE [m] — 전체 표는 `results/metrics_simulink.txt`

| 경로 | Stanley kin | Stanley dyn | PD kin | PD dyn |
|---|---|---|---|---|
| 직선 | 0.186 | 0.200 | 0.334 | 0.344 |
| 원형 R=100 | 0.105 | 0.179 | 1.427 | 1.883 |
| 우회전 | 0.130 | 0.247 | 1.908 | 2.532 |
| 레이스트랙 | 0.139 | 0.273 | 2.077 | 2.780 |

곡률 피드포워드 (dynamic, k = 2): 레이스트랙 RMSE 0.273 → 0.094 m (50 km/h), 0.548 → 0.100 m (70 km/h). 원형 잔류 오차 −0.162 → 0.003 m.

게인: kinematic k ≤ 3 안정, dynamic은 k = 3이 경계(직선 발산·레이스트랙 수렴), k = 4는 둘 다 발산.
원인은 초기 오차에서 조향 명령이 레이트 제한(500 deg/s)에 걸리는 비선형 포화. Rate Limiter를 풀거나 초기 오차를 0.1 m로 줄이면 k = 8도 수렴. 기본값 k = 2.

## Python 재현 시뮬레이터

`sim/sim.py` — 같은 블록 순서를 옮긴 것. Simulink 결과와 RMSE가 소수 셋째 자리까지 일치(dynamic k = 3 경계만 다름). MATLAB 없이 빠른 게인 실험용.
`sim/plot_simulink.py stanley_eval.mat results` — Simulink 결과로 그림 생성.
