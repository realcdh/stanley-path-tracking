# Stanley 경로 추종 제어기 (PBL)

Simulink `PathTracking` 모델의 Path Tracking Controller 빈 슬롯(Multiport Switch 2번 데이터 포트)에 넣는 Stanley 제어기와, 결과 분석용 Python 재현 시뮬레이터.

배포 모델·경로 데이터(`.slx`, `path_data_*.mat`)는 수업 자료라 포함하지 않음.

## 제어 법칙

```
e_fa  = e_y + Lf·sin(e_ψ)                    % 무게중심 오차 -> 전륜축 오차
δ     = -( e_ψ + atan( k·e_fa / (v_x + ε) ) )
```
부호: 모델 규약상 `e_y > 0`(차량이 경로 좌측), `e_ψ > 0`(좌측을 향함)일 때 우조향(δ < 0).

## MATLAB 적용

```matlab
addpath('<이 리포>/matlab')
add_stanley_block('<배포 폴더>/실습 1차 배포 버전(261111)')   % 1회
run_simulation                                                % Controller_mode = 2
```
`add_stanley_block` 동작: 원본 백업(`*_backup.slx`) → MATLAB Function 블록 추가·연결 → `Controller` 상수를 `Controller_mode`로 변경 → `init_param.m` 끝에 `stanley_param;` 추가.
MATLAB 없는 환경에서 작성해 실행 검증 안 됨. 실패 시 수동: `matlab/Stanley_Control.m` 내용을 MATLAB Function 블록에 붙이고, `k_st`·`eps_st`·`Lf_st`를 Parameter로 지정, Multiport Switch 입력 2개로 변경 후 3번 포트에 연결.

## Python 재현 시뮬레이션

```bash
python sim/sim.py "<배포 폴더 경로>" results
```
배포 모델 블록 순서(Error Calculator 코드 이식, 조향 액추에이터 지연·포화·레이트 제한, kinematic/dynamic bicycle model, Ts = 0.01 s, 50 km/h 등속)를 그대로 따름. Simulink 결과와 수치가 조금 다를 수 있음.

| 결과 (레이스트랙, kinematic) | 횡오차 RMSE [m] |
|---|---|
| Stanley k = 2 | 0.139 |
| PD (배포 기본) | 2.077 |

게인: k ≤ 2.5 안정, k = 3부터 dynamic 모델 발산, k = 4는 두 모델 모두 발산. 원인은 초기 1 m 오차에서 조향 명령이 레이트 제한(500 deg/s)에 걸리는 비선형 포화(레이트 제한 제거 시 k = 8도 수렴). 기본값 k = 2.
