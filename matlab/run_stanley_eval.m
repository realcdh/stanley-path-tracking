%% run_stanley_eval.m  경로 4종 x 차량모델 2종 x 제어기 2종 + 게인 스윕, Simulink 일괄 실행
% add_stanley_block 적용한 배포 폴더를 현재 폴더로 두고 실행 (결과: stanley_eval.mat)
% 주의: 현재 폴더·MATLAB 경로에 한글 폴더가 있으면 MATLAB Function 블록 컴파일 실패
clear; close all; clc
mdl = 'PathTracking_26_02_261111';
load_system(mdl);
names = {'linear','circular','right_turn','racetrack'};
rmse  = @(x) sqrt(mean(x.^2));

rows = {}; logs = struct();
for Path_mode = 1:4
    for vm = 1:2                    % 1 : kinematic / 2 : dynamic
        for cm = [2 1]              % 2 : Stanley / 1 : PID 블록(배포 기본 PD)
            o = run1(mdl, Path_mode, vm, cm, 2.0);
            last5 = o.t >= o.t(end) - 5;
            rows(end+1, :) = {names{Path_mode}, vm, cm, rmse(o.e_y), max(abs(o.e_y)), ...
                rad2deg(rmse(o.e_yaw)), max(abs(o.Ay)), rmse(o.e_y(last5)), o.t(end)}; %#ok<SAGROW>
            logs.(sprintf('%s_m%d_c%d', names{Path_mode}, vm, cm)) = o;
        end
    end
end

sweep = {};
for vm = 1:2
    for kk = [0.5 1 1.5 2 2.5 3 4]
        for Path_mode = [1 4]
            o = run1(mdl, Path_mode, vm, 2, kk);
            bad = find(abs(o.e_y) >= 0.05, 1, 'last');
            ts = NaN; if bad < numel(o.t), ts = o.t(bad+1); end
            sweep(end+1, :) = {names{Path_mode}, vm, kk, rmse(o.e_y), max(abs(o.Ay)), ts, -min(o.e_y)}; %#ok<SAGROW>
            if Path_mode == 1 && vm == 1
                logs.(sprintf('step_k%02d', round(kk*10))) = o;
            end
        end
    end
end
close_system(mdl, 0);

T = cell2table(rows, 'VariableNames', {'path','vehicle','ctrl','rmse_ey','max_ey','rmse_eyaw_deg','max_ay','ey_last5','t_end'});
S = cell2table(sweep, 'VariableNames', {'path','vehicle','k','rmse_ey','max_ay','settle_5cm','overshoot'});
disp(T); disp(S);
save('stanley_eval.mat', 'T', 'S', 'logs');


function o = run1(mdl, Path_mode, vm, cm, k)
% 모델 InitFcn 이 기본 작업공간에서 init_param 을 다시 부르므로 변수는 base 에 둔다
    vals = {'Path_mode', Path_mode; 'ey0', 1.0; 'ephi0', 0; 'k_st', k; 'Controller_mode', cm; ...
            'sim_time', 50.0; 'Ts', 0.01};
    for i = 1:size(vals, 1), assignin('base', vals{i, 1}, vals{i, 2}); end
    evalin('base', 'init_param;');
    in = Simulink.SimulationInput(mdl);
    in = in.setBlockParameter([mdl '/Vis_on'], 'Value', 'false');
    in = in.setBlockParameter([mdl '/2DOF Bicycle  Model/Vehicle Model'], 'Value', num2str(vm));
    out = sim(in);
    o = struct('t', out.get('tout'), 'X', out.get('X_msr'), 'Y', out.get('Y_msr'), 'e_y', out.get('e_y'), ...
               'e_yaw', out.get('e_yaw'), 'SWA_act', out.get('SWA_act_rad'), 'Ay', out.get('Ay'), ...
               'curv', out.get('curvature'), 'Path', evalin('base', 'Path'));
end
