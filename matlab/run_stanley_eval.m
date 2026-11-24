%% run_stanley_eval.m  Stanley 평가 일괄 실행 (Simulink)
% add_stanley_block 적용한 배포 폴더를 현재 폴더로 두고 실행 (결과: stanley_eval.mat)
% 주의: 현재 폴더·MATLAB 경로에 한글 폴더가 있으면 MATLAB Function 블록 컴파일 실패
%   T  : 경로 4종 x 차량모델 2종 x {Stanley, Stanley+FF, PD}
%   S  : 게인 스윕 (순수 Stanley, 직선·레이스트랙)
%   V  : 속도 스윕 (원형·레이스트랙, 30/50/70 km/h, FF 유무) + 원형 정상상태 오차 이론값
clear; close all; clc
mdl = 'PathTracking_26_02_261111';
load_system(mdl);
names = {'linear','circular','right_turn','racetrack'};
rmse  = @(x) sqrt(mean(x.^2));
P = struct('L', 2.7, 'lf', 1.08, 'lr', 1.62, 'm', 1573, 'Cf', 1.6e5);   % 정상상태 이론값용

rows = {}; logs = struct();
cfg = {'stanley', 2, 0; 'stanley_ff', 2, 1; 'pd', 1, 0};
for Path_mode = 1:4
    for vm = 1:2                                % 1 : kinematic / 2 : dynamic
        for c = 1:size(cfg, 1)
            o = run1(mdl, Path_mode, vm, cfg{c, 2}, 2.0, cfg{c, 3}, 50/3.6);
            last5 = o.t >= o.t(end) - 5;
            rows(end+1, :) = {names{Path_mode}, vm, cfg{c, 1}, rmse(o.e_y), max(abs(o.e_y)), ...
                rad2deg(rmse(o.e_yaw)), max(abs(o.Ay)), rmse(o.e_y(last5)), o.t(end)}; %#ok<SAGROW>
            logs.(sprintf('%s_m%d_%s', names{Path_mode}, vm, cfg{c, 1})) = o;
        end
    end
end

sweep = {};
for vm = 1:2
    for kk = [0.5 1 1.5 2 2.5 3 4]
        for Path_mode = [1 4]
            o = run1(mdl, Path_mode, vm, 2, kk, 0, 50/3.6);
            bad = find(abs(o.e_y) >= 0.05, 1, 'last');
            ts = NaN; if bad < numel(o.t), ts = o.t(bad+1); end
            sweep(end+1, :) = {names{Path_mode}, vm, kk, rmse(o.e_y), max(abs(o.Ay)), ts, -min(o.e_y)}; %#ok<SAGROW>
            if Path_mode == 1 && vm == 1
                logs.(sprintf('step_k%02d', round(kk*10))) = o;
            end
        end
    end
end

spd = {};
for v_kph = [30 50 70]
    v = v_kph/3.6;
    for vm = 1:2
        for ff = [0 1]
            oc = run1(mdl, 2, vm, 2, 2.0, ff, v);              % 원형 R = 100 m
            ss = oc.t >= oc.t(end) - 10;
            kap = 0.01; beta_ss = P.lr*kap - (vm == 2)*P.m*P.lf*v^2*kap/(P.L*P.Cf);
            e_fa = -(v + 1)/2*kap*(P.lf + (vm == 2)*P.m*P.lr*v^2/(P.L*P.Cf));
            e_th = (1 - ff)*(e_fa + P.lf*sin(beta_ss));          % 무게중심 기준 이론 정상상태 오차
            orr = run1(mdl, 4, vm, 2, 2.0, ff, v);             % 레이스트랙
            spd(end+1, :) = {v_kph, vm, ff, mean(oc.e_y(ss)), e_th, rmse(orr.e_y), max(abs(orr.e_y)), max(abs(orr.Ay))}; %#ok<SAGROW>
        end
    end
end
close_system(mdl, 0);

T = cell2table(rows, 'VariableNames', {'path','vehicle','ctrl','rmse_ey','max_ey','rmse_eyaw_deg','max_ay','ey_last5','t_end'});
S = cell2table(sweep, 'VariableNames', {'path','vehicle','k','rmse_ey','max_ay','settle_5cm','overshoot'});
V = cell2table(spd, 'VariableNames', {'v_kph','vehicle','ff','circ_ss_ey','circ_ss_theory','race_rmse','race_max_ey','race_max_ay'});
disp(T); disp(S); disp(V);
save('stanley_eval.mat', 'T', 'S', 'V', 'logs');


function o = run1(mdl, Path_mode, vm, cm, k, ff, v0)
% 모델 InitFcn 이 기본 작업공간에서 init_param 을 다시 부르므로 변수는 base 에 둔다
    vals = {'Path_mode', Path_mode; 'ey0', 1.0; 'ephi0', 0; 'k_st', k; 'ff_st', ff; ...
            'Controller_mode', cm; 'sim_time', 50.0; 'Ts', 0.01};
    for i = 1:size(vals, 1), assignin('base', vals{i, 1}, vals{i, 2}); end
    evalin('base', 'init_param;');
    in = Simulink.SimulationInput(mdl);
    in = in.setBlockParameter([mdl '/Vis_on'], 'Value', 'false');
    in = in.setBlockParameter([mdl '/2DOF Bicycle  Model/Vehicle Model'], 'Value', num2str(vm));
    assignin('base', 'v_run', v0);                                % 등속 주행 속도 (Vx0 · Vx_max 대신)
    vi = [mdl '/2DOF Bicycle  Model/Discrete-Time' newline 'Integrator1'];
    in = in.setBlockParameter(vi, 'InitialCondition', 'v_run');
    in = in.setBlockParameter(vi, 'UpperSaturationLimit', 'v_run');
    out = sim(in);
    o = struct('t', out.get('tout'), 'X', out.get('X_msr'), 'Y', out.get('Y_msr'), 'e_y', out.get('e_y'), ...
               'e_yaw', out.get('e_yaw'), 'SWA_act', out.get('SWA_act_rad'), 'Ay', out.get('Ay'), ...
               'curv', out.get('curvature'), 'Path', evalin('base', 'Path'));
end
