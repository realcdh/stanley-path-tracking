function delta = Stanley_Control(e_y, e_yaw, vx, curv, k_st, eps_st, Lf_st, Kv_st, ff_st)
%STANLEY_CONTROL  Stanley 경로 추종 제어기 + 곡률 피드포워드 (MATLAB Function 블록 본문)
%   e_y    : 횡방향 오차 [m]   (무게중심 기준, 차량이 경로 좌측이면 +)
%   e_yaw  : 헤딩 오차 [rad]   (차량이 경로보다 좌측을 향하면 +)
%   vx     : 종방향 속도 [m/s]
%   curv   : 경로 곡률 [1/m]   (좌선회 +)
%   k_st   : 횡오차 게인 [1/s]
%   eps_st : 저속 발산 방지 속도 [m/s]
%   Lf_st  : 무게중심~전륜축 거리 [m]
%   Kv_st  : 슬립 보상 계수 m*lr/(L*Cf) [s^2/m]   (Cf : 전륜 축 코너링 강성)
%   ff_st  : 1 = 곡률 피드포워드 사용, 0 = 순수 Stanley
%   delta  : 전륜 조향각 [rad] (좌회전 +)
%#codegen

    % Stanley는 전륜축 기준 오차를 쓴다. Error Calculator는 무게중심 기준이므로 변환.
    % ponytail: 곡률 항(Lf^2*kappa/2) 생략, R=50 m에서 약 1 cm 오차
    e_fa = e_y + Lf_st*sin(e_yaw);

    % 순수 Stanley : 부호 규약상 오차가 +이면 우회전(-) 조향
    delta = -(e_yaw + atan2(k_st*e_fa, vx + eps_st));

    % 정상상태 원선회에서 순수 Stanley는 e_fa = -(v+eps)/k * kappa*(Lf + Kv*v^2) 만큼 바깥으로 밀림.
    % 그만큼을 미리 더해 정상상태 오차를 0으로 만든다 (2DOF 선형 bicycle 모델 기준 유도).
    delta = delta + ff_st * curv * (Lf_st + Kv_st*vx^2);
end
