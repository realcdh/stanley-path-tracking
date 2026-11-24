%% Stanley Control 파라미터
% init_param.m 맨 끝에서 `stanley_param;` 로 호출 (add_stanley_block 이 자동 추가)
Controller_mode = 2;        % 1 : PID Control / 2 : Stanley Control
k_st   = 2.0;               % 횡오차 게인 [1/s]  (Python 재현 기준 k >= 3 이면 dynamic 모델 발산, 조향 레이트 제한 포화)
eps_st = 1.0;               % 저속 발산 방지 [m/s]
Lf_st  = VehicleParam.Lf;   % 무게중심~전륜축 [m] (1.08 m)
