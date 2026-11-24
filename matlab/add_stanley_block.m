function add_stanley_block(dist_dir)
%ADD_STANLEY_BLOCK  배포 모델에 Stanley Control 블록을 삽입하거나 최신 코드로 갱신
%   add_stanley_block('<배포 폴더>')     % 여러 번 실행해도 됨
%
%   1) 원본 .slx 를 *_backup.slx 로 복사 (최초 1회)
%   2) Path Tracking Controller 안에 MATLAB Function 블록 'Stanley Control' 추가 또는 코드 갱신
%      (k_st, eps_st, Lf_st, Kv_st, ff_st 는 작업공간 Parameter)
%   3) From(e_y, e_yaw, vx, curv) -> Stanley -> Multiport Switch 3번 포트(데이터 2번) 연결 (빠진 것만)
%   4) Controller 상수를 변수 Controller_mode 로 바꿈 (1=PID, 2=Stanley)
%   5) stanley_param.m 을 배포 폴더에 복사, init_param.m 끝에 호출 추가
%   주의: 시뮬레이션은 한글 없는 경로에서 실행해야 함 (MATLAB Function 블록 컴파일 실패)

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

    % --- MATLAB Function 블록 (없으면 생성)
    if getSimulinkBlockHandle(blk) < 0
        p = get_param([sys '/PID Control'], 'Position');
        add_block('simulink/User-Defined Functions/MATLAB Function', blk, 'Position', p + [0 160 0 160]);
    end
    pos = get_param(blk, 'Position');
    ch = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', blk);
    ch.Script = fileread(fullfile(here, 'Stanley_Control.m'));
    for n = {'k_st', 'eps_st', 'Lf_st', 'Kv_st', 'ff_st'}
        d = ch.find('-isa', 'Stateflow.Data', 'Name', n{1});
        d.Scope = 'Parameter';
    end

    % --- 입력 From 블록 : 연결 안 된 포트만 (Goto 태그는 서브시스템 안에 이미 있음)
    tags = {'e_y', 'e_yaw', 'vx', 'curv'};
    pc = get_param(blk, 'PortConnectivity');
    for i = 1:numel(tags)
        if pc(i).SrcBlock ~= -1, continue; end
        fb = sprintf('%s/From_st_%s', sys, tags{i});
        if getSimulinkBlockHandle(fb) < 0
            add_block('simulink/Signal Routing/From', fb, 'GotoTag', tags{i}, ...
                'Position', [pos(1)-120, pos(2)+20*(i-1), pos(1)-60, pos(2)+20*(i-1)+14]);
        end
        add_line(sys, sprintf('From_st_%s/1', tags{i}), sprintf('Stanley Control/%d', i), ...
            'autorouting', 'on');
    end

    % --- 스위치 포트 확장 + 연결 (최초 1회)
    if str2double(get_param(sw, 'Inputs')) < 2
        set_param(sw, 'Inputs', '2');
        add_line(sys, 'Stanley Control/1', ['Multiport' newline 'Switch/3'], 'autorouting', 'on');
    end
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
    fprintf('완료: %s 의 Stanley Control 추가/갱신. run_simulation 실행\n', mdl);
end
