function add_stanley_block(dist_dir)
%ADD_STANLEY_BLOCK  배포 모델에 Stanley Control 블록을 삽입 (1회 실행)
%   add_stanley_block('…/실습 1차 배포 버전(261111)')
%
%   1) 원본 .slx 를 *_backup.slx 로 복사
%   2) Path Tracking Controller 안에 MATLAB Function 블록 'Stanley Control' 추가
%      (k_st, eps_st, Lf_st 는 작업공간 Parameter)
%   3) From(e_y, e_yaw, vx) -> Stanley -> Multiport Switch 3번 포트(데이터 2번) 연결
%   4) Controller 상수를 변수 Controller_mode 로 바꿈 (1=PID, 2=Stanley)
%   5) stanley_param.m 을 배포 폴더에 복사, init_param.m 끝에 호출 추가

    here = fileparts(mfilename('fullpath'));
    mdl  = 'PathTracking_26_02_261111';
    f    = fullfile(dist_dir, [mdl '.slx']);
    bak  = fullfile(dist_dir, [mdl '_backup.slx']);
    if ~isfile(bak), copyfile(f, bak); end

    cd(dist_dir);
    load_system(mdl);
    sys = [mdl '/Path Tracking Controller'];
    blk = [sys '/Stanley Control'];
    sw  = [sys '/Multiport' newline 'Switch'];
    if getSimulinkBlockHandle(blk) > 0
        error('Stanley Control 블록이 이미 있음. 백업(%s)에서 복원 후 재실행', bak);
    end

    % --- MATLAB Function 블록
    p = get_param([sys '/PID Control'], 'Position');
    pos = p + [0 160 0 160];
    add_block('simulink/User-Defined Functions/MATLAB Function', blk, 'Position', pos);
    ch = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', blk);
    ch.Script = fileread(fullfile(here, 'Stanley_Control.m'));
    for n = {'k_st', 'eps_st', 'Lf_st'}
        d = ch.find('-isa', 'Stateflow.Data', 'Name', n{1});
        d.Scope = 'Parameter';
    end

    % --- 입력 From 블록 (Goto 태그는 서브시스템 안에 이미 있음)
    tags = {'e_y', 'e_yaw', 'vx'};
    for i = 1:3
        fb = sprintf('%s/From_st_%s', sys, tags{i});
        add_block('simulink/Signal Routing/From', fb, 'GotoTag', tags{i}, ...
            'Position', [pos(1)-120, pos(2)+20*(i-1), pos(1)-60, pos(2)+20*(i-1)+14]);
        add_line(sys, sprintf('From_st_%s/1', tags{i}), sprintf('Stanley Control/%d', i), ...
            'autorouting', 'on');
    end

    % --- 스위치 포트 확장 + 연결
    set_param(sw, 'Inputs', '2');
    add_line(sys, 'Stanley Control/1', ['Multiport' newline 'Switch/3'], 'autorouting', 'on');
    set_param([sys '/Controller'], 'Value', 'Controller_mode');

    save_system(mdl);
    close_system(mdl);

    % --- 파라미터 스크립트 연결
    copyfile(fullfile(here, 'stanley_param.m'), dist_dir);
    ip = fullfile(dist_dir, 'init_param.m');
    if ~contains(fileread(ip), 'stanley_param')
        fid = fopen(ip, 'a');
        fprintf(fid, '\n%%%% Stanley Control\nstanley_param;\n');
        fclose(fid);
    end
    fprintf('완료: %s 에 Stanley Control 추가. run_simulation 실행\n', mdl);
end
