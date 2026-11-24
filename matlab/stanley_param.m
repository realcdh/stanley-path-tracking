%% Stanley Control 파라미터
% init_param.m 맨 끝에서 `stanley_param;` 로 호출 (add_stanley_block 이 자동 추가)
% 모델 InitFcn 이 시뮬레이션마다 init_param 을 다시 부르므로, 이미 있는 값은 덮어쓰지 않음
% (run_simulation 은 처음에 clear all 하므로 매 실행 아래 기본값으로 시작)
if ~exist('Controller_mode', 'var'), Controller_mode = 2;   end  % 1 : PID Control / 2 : Stanley Control
if ~exist('k_st', 'var'),            k_st   = 2.0;          end  % 횡오차 게인 [1/s] (k >= 3 이면 dynamic 모델 발산, 조향 레이트 제한 포화)
if ~exist('eps_st', 'var'),          eps_st = 1.0;          end  % 저속 발산 방지 [m/s]
Lf_st = VehicleParam.Lf;                                         % 무게중심~전륜축 [m] (1.08 m)
