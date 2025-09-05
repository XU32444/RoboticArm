
% Robotic Arm Control UI in MATLAB for 6-DOF Arm using MIT Mode
% not sure if kp kd is working, the frozen time isnt working neither
function robotic_arm_ui
    % Create UI figure
    fig = uifigure('Name','6-DOF Robotic Arm UI','Position',[50 50 1400 800]);

    % Global storage for motors
    motors = cell(1, 6);
    mc = [];  % MotorControl object
    target_positions = zeros(1, 6); % radians
    sliders = gobjects(1,6);
    edits = gobjects(1,6);
    kp_vals = ones(1,6);   kd_vals  = ones(1,6);
    vdes_vals = zeros(1,6);          % NEW
    tauff_vals = zeros(1,6);         % NEW
    
    kp_fields = gobjects(1,6); kd_fields = gobjects(1,6);
    v_fields  = gobjects(1,6); tau_fields = gobjects(1,6);   % NEW


    % Repetition test state
    rep_start_vals = zeros(1,6);
    rep_end_vals = zeros(1,6);
    rep_active = false;
    rep_paused = false;
    rep_state = "idle";
    rep_timer = [];
    frozen_period = 1.0;
    rep_next_time = 0;

    try
        addpath('.\DM_CAN\');
        motor_types = [DM_Motor_Type.DM4340, DM_Motor_Type.DM4340, DM_Motor_Type.DM4340, ...
                       DM_Motor_Type.DM4310, DM_Motor_Type.DM4310, DM_Motor_Type.DM4310];
        slave_ids = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06];
        master_ids = [0x11, 0x12, 0x13, 0x14, 0x15, 0x16];
        for i = 1:6
            motors{i} = Motor(motor_types(i), slave_ids(i), master_ids(i));
        end
        mc = MotorControl('COM4', 921600);

        for i = 1:6
            mc.addMotor(motors{i});
            mc.switchControlMode(motors{i}, Control_Type.MIT);
        end
        for i = 1:6
            mc.enable(motors{i});
        end
    catch e
        uialert(fig, e.message, 'Initialization Error');
        return;
    end

for i = 1:6
    y_offset = 520 - (i-1)*70;

    % ---- Position target ----
    uilabel(fig,'Text',sprintf('M%d Target (°)',i),'Position',[50 y_offset 100 22]);

    sliders(i) = uislider(fig,'Position',[150 y_offset+10 400 3], ...
                 'Limits',[-720 720],'Value',0, ...
                 'ValueChangedFcn',@(s,evt) slider_update_callback(i,evt.Value));

    edits(i) = uieditfield(fig,'numeric','Position',[570 y_offset 60 22], ...
                'Limits',[-720 720],'Value',0, ...
                'ValueChangedFcn',@(e,evt) edit_update_callback(i,evt.Value));

    uibutton(fig,'Text','Zero','Position',[640 y_offset 50 22], ...
             'ButtonPushedFcn',@(btn,evt) set_zero(i));

    % ---- KP / KD ----
    uilabel(fig,'Text','KP','Position',[700 y_offset 25 22]);
    kp_fields(i) = uieditfield(fig,'numeric','Position',[725 y_offset 45 22], ...
                     'Limits',[0 500],'Value',kp_vals(i), ...
                     'ValueChangedFcn',@(src,~) set_kp(i,src.Value));

    uilabel(fig,'Text','KD','Position',[775 y_offset 25 22]);
    kd_fields(i) = uieditfield(fig,'numeric','Position',[800 y_offset 45 22], ...
                     'Limits',[0 5],'Value',kd_vals(i), ...
                     'ValueChangedFcn',@(src,~) set_kd(i,src.Value));

    % ---- Vdes / TauFF ----
    uilabel(fig,'Text','Vdes','Position',[850 y_offset 35 22]);
    v_fields(i) = uieditfield(fig,'numeric','Position',[885 y_offset 45 22], ...
                     'Limits',[-100 100],'Value',vdes_vals(i), ...
                     'ValueChangedFcn',@(src,~) set_vdes(i,src.Value));

    uilabel(fig,'Text','τ_ff','Position',[935 y_offset 35 22]);
    tau_fields(i) = uieditfield(fig,'numeric','Position',[970 y_offset 45 22], ...
                     'Limits',[-5 5],'Value',tauff_vals(i), ...
                     'ValueChangedFcn',@(src,~) set_tauff(i,src.Value));
end


    uibutton(fig, 'Text','Send Command','Position',[350 50 120 30], 'ButtonPushedFcn', @(btn,event) send_command());
    uibutton(fig, 'Text','Exit','Position',[500 50 80 30], 'ButtonPushedFcn', @(btn,event) onExit());

    posText = uitextarea(fig, 'Position',[1080 200 200 360], 'Editable','off');

% ----------  Repetition panel  ----------
panel_h = 200;               % plenty of room for 6 rows
rep_panel = uipanel(fig,'Title','Repetition',...
                    'Position',[400 560 430 panel_h]);
% Frozen-period edit
uilabel(rep_panel,'Text','Frozen (s):','Position',[10 panel_h-40 70 22]);
uieditfield(rep_panel,'numeric','Value',frozen_period,...
            'Position',[85 panel_h-40 60 22],...
            'Limits',[0 10],'ValueChangedFcn',@(src,~) set_freeze(src.Value));

% Column headers
uilabel(rep_panel,'Text','Pi°','Position',[155 panel_h-40 40 22]);
uilabel(rep_panel,'Text','Pf°','Position',[205 panel_h-40 40 22]);

% Six rows of Pi / Pf
base_y = panel_h-70;   row_h = 30;
for i = 1:6
    y = base_y - (i-1)*row_h;
    uilabel(rep_panel,'Text',sprintf('M%d',i),'Position',[10 y 30 22]);

    % Pi
    uieditfield(rep_panel,'numeric','Position',[150 y 45 22],...
        'Limits',[-720 720],...
        'ValueChangedFcn',@(src,~) set_rep_val(i,1,src.Value));

    % Pf
    uieditfield(rep_panel,'numeric','Position',[200 y 45 22],...
        'Limits',[-720 720],...
        'ValueChangedFcn',@(src,~) set_rep_val(i,2,src.Value));
end

% Start / Pause / Reset
uibutton(rep_panel,'Text','Start','Position',[330 panel_h-40 70 26],...
         'ButtonPushedFcn',@(btn,~) start_repetition());
uibutton(rep_panel,'Text','Pause','Position',[330 panel_h-75 70 26],...
         'ButtonPushedFcn',@(btn,~) pause_repetition());
uibutton(rep_panel,'Text','Reset','Position',[330 panel_h-110 70 26],...
         'ButtonPushedFcn',@(btn,~) reset_repetition());
% ----------------------------------------

    t = timer('ExecutionMode','fixedRate','Period',0.1,'TimerFcn',@refresh_status); start(t);
    rep_timer = timer('ExecutionMode','fixedRate','Period',0.1,'TimerFcn',@repetition_step);

    fig.CloseRequestFcn = @(src, event) onClose();

    function slider_update_callback(i, deg)
        edits(i).Value = deg;
        target_positions(i) = deg2rad(deg);
    end

    function edit_update_callback(i, deg)
        sliders(i).Value = deg;
        target_positions(i) = deg2rad(deg);
    end

    function send_command()
        for i = 1:6
            mc.controlMIT(motors{i}, ...
                kp_vals(i), kd_vals(i), ...
                target_positions(i), vdes_vals(i), tauff_vals(i));
        end
    end


    function set_kp(idx,val),    kp_vals(idx)   = val; end
    function set_kd(idx,val),    kd_vals(idx)   = val; end
    function set_vdes(idx,val),  vdes_vals(idx) = val; end
    function set_tauff(idx,val), tauff_vals(idx)= val; end
    %% =============== 手动施教 / 轨迹复现 ==================
% ---- 1. 运行时需要的“全局”变量 ------------------------
teach_active   = false;      % 是否正在记录
teach_buf      = [];         % 轨迹缓存 [t  q1 … q6]
teach_t0       = 0;          % teach 起始时间
teach_timer    = [];         % 20 ms 采样
play_timer     = [];         % 20 ms 播放
play_idx       = 1;          % 正在播放的帧号

% ---- 2. 三个按钮：Teach (开始/结束) + Replay 开始 + Replay 停止 --
teachBtn = uibutton(fig,"Text","Teach (Start)", ...
        "Position",[750 50 110 30], ...
        "ButtonPushedFcn",@toggle_teach);

replayBtn = uibutton(fig,"Text","Replay", ...
        "Position",[875 50 80 30], ...
        "ButtonPushedFcn",@start_replay);

stopReplayBtn = uibutton(fig,"Text","Stop Replay", ...
        "Position",[970 50 90 30], ...
        "ButtonPushedFcn",@stop_replay);

%% ---------- 3. Teach：开始/结束 ---------------------------------
function toggle_teach(src,~)
    if ~teach_active        % ====== 准备开始 Teach ======
        teach_active = true;
        teach_buf  = [];           % 清空旧数据
        teach_t0   = tic;
        % 让电机“松软”——kp=kd=0，使人手易于拖动
        for ii = 1:6
            mc.controlMIT(motors{ii},0,0,motors{ii}.getPosition,0,0);
        end
        % 定时采集 50 Hz（Period=0.02 s）
        teach_timer = timer("ExecutionMode","fixedRate", ...
                            "Period",0.02,"TimerFcn",@collect_teach);
        start(teach_timer);
        src.Text = "Teach (End)";
    else                    % ====== 结束 Teach ==========
        teach_active = false;
        stop(teach_timer);  delete(teach_timer);
        src.Text = "Teach (Start)";
        % 这里 teach_buf 已经拿到完整轨迹，后续 replay 用
        uialert(fig,sprintf("轨迹记录完毕，共 %d 帧。",size(teach_buf,1)), ...
                     "Teach 结束");
    end
end

% ---------- 4. Teach 采样回调 -------------------------------------
function collect_teach(~,~)
    tnow = toc(teach_t0);
    qrow = zeros(1,6);
    for ii = 1:6
        mc.refresh_motor_status(motors{ii});
        qrow(ii) = motors{ii}.getPosition();  % rad
    end
    teach_buf(end+1,:) = [tnow qrow];
end
function move_to_start()
    start_pos = teach_buf(1,2:end);      % rad
    start_pos_deg = rad2deg(start_pos);  % 如果库要弧度就保留原值

    for ii = 1:6
        % 给 20 °/s 的前馈速度以防 0 速度不响应
        mc.control_Pos_Vel(motors{ii}, start_pos_deg(ii), 20); 
    end

    % 简单等 0.5~1 s；如果想精确到位可继续做误差检测
    pause(1.0);
end


%% ---------- 5. Replay：开始 / 停止 -------------------------------
%% =====  start_replay：切到 POS_VEL，并启动定时器  =====
function start_replay(~,~)
    if isempty(teach_buf)
        uialert(fig,"还没有 Teach 轨迹！","提示");  return;
    end

    % (1) 清理旧计时器
    if ~isempty(play_timer)
        try, stop(play_timer); end
        if isvalid(play_timer), delete(play_timer); end
        play_timer = [];
    end

    % (2) 切到 POS_VEL 并重新 Enable
    for ii = 1:6
        mc.switchControlMode(motors{ii}, Control_Type.POS_VEL);
        pause(0.005);                          % 有些固件需要一点间隔
        mc.enable(motors{ii});                 % <—— 关键
    end
    pause(0.05);                              % 让固件完全就绪

    % (3) 先把手臂移到“Teach 第一帧”
    move_to_start();

    % (4) 启动循环回放
    play_idx   = 1;
    play_timer = timer("ExecutionMode","fixedRate", ...
                       "Period",0.02, ...
                       "TimerFcn",@play_step);
    start(play_timer);
end






%% =====  stop_replay：停止计时器，切回 MIT  =====
function stop_replay(~,~)
    if ~isempty(play_timer)
        try, stop(play_timer); end
        if isvalid(play_timer), delete(play_timer); end
        play_timer = [];
    end
    % 如前：切回 MIT
    for ii = 1:6
        mc.switchControlMode(motors{ii}, Control_Type.MIT);
    end
end


% ---------- 6. 每 20 ms 播放一帧 -------------------------------
%% =====  play_step：POS_VEL 指令  =====
play_idx   = 2;  
function play_step(~,~)
    if isempty(teach_buf), return; end

    cur_idx  = play_idx;
    next_idx = play_idx + 1;
    if next_idx > size(teach_buf,1)
        % 到最后一帧→多停 1 周期再回第 1 帧，避免跳变
        next_idx = 1;    % 环回
        pause(0.02);     % 空一个周期
    end

    q  = teach_buf(cur_idx,2:end);               % 位置(rad)
    dq = (teach_buf(next_idx,2:end) - q)/0.02;   % 简单双差分

    % ---------- 速度限幅 ----------
    dq = max(min(dq, 3.0), -3.0);   % ±3 rad/s 上限
    % ------------------------------

    for ii = 1:6
        mc.control_Pos_Vel(motors{ii}, q(ii), dq(ii));
    end

    play_idx = next_idx;
end

% function play_step(~,~)
%     if isempty(teach_buf), return; end
% 
%     % 位置、速度（rad → deg）
%     pos_rad   = teach_buf(play_idx,2:end);
%     next_idx  = play_idx + 1;
%     if next_idx > size(teach_buf,1), next_idx = 1; end
%     vel_rad   = (teach_buf(next_idx,2:end) - pos_rad)/0.02;
% 
%     pos_deg = rad2deg(pos_rad);
%     vel_deg = rad2deg(vel_rad);
% 
%     % 发送
%     for ii = 1:6
%         % mc.control_Pos_Vel(motors{ii}, pos_deg(ii), vel_deg(ii));
%         mc.control_Pos_Vel(motors{ii}, pos_deg(ii), 0);
%     end
%     play_idx = next_idx;
% end


%% ================================================================

    function set_zero(i)
        try
            mc.set_zero_position(motors{i});
            sliders(i).Value = 0;
            edits(i).Value = 0;
            target_positions(i) = 0;
            uialert(fig, sprintf('Motor %d zero set.', i), 'Success');
        catch e
            uialert(fig, e.message, sprintf('Set Zero Failed for Motor %d', i));
        end
    end

    function refresh_status(~,~)
        try
            out = cell(6,1);
            for i = 1:6
                mc.refresh_motor_status(motors{i});
                p  = motors{i}.getPosition()*180/pi;   % deg
                v  = motors{i}.getVelocity();
                tq = motors{i}.getTorque();
                out{i} = sprintf('M%d  P=%6.1f°  V=%6.2f  τ=%5.2f  KP=%4.1f KD=%3.2f  Vd=%5.1f τff=%4.2f',...
                                  i,p,v,tq, kp_vals(i),kd_vals(i), vdes_vals(i),tauff_vals(i));
            end
            posText.Value = out;          % must be cell array or string array
        catch err
            posText.Value = {['Status error: ' err.message]};
        end
    end


    function set_rep_val(idx, pos, deg)
        if pos == 1
            rep_start_vals(idx) = deg2rad(deg);
        else
            rep_end_vals(idx) = deg2rad(deg);
        end
    end

    function start_repetition()
        rep_active = true;
        rep_paused = false;
        rep_state = "move_to_B";
        rep_next_time = tic;
        start(rep_timer);
    end

    function pause_repetition()
        rep_paused = ~rep_paused;
    end

    function reset_repetition()
        rep_active = false;
        rep_paused = false;
        rep_state = "idle";
        stop(rep_timer);
    end

    function repetition_step(~,~)
        if ~rep_active || rep_paused, return; end
        elapsed = toc(rep_next_time);
        switch rep_state
            case "move_to_A"
                for i = 1:6
                    mc.controlMIT(motors{i}, kp_vals(i), kd_vals(i), rep_start_vals(i), vdes_vals(i), tauff_vals(i));
                end
                rep_state = "wait_A";
                rep_next_time = tic;
            case "wait_A"
                if elapsed >= frozen_period
                    rep_state = "move_to_B";
                end
            case "move_to_B"
                for i = 1:6
                    mc.controlMIT(motors{i}, kp_vals(i), kd_vals(i), rep_end_vals(i), vdes_vals(i), tauff_vals(i));
                end
                rep_state = "wait_B";
                rep_next_time = tic;
            case "wait_B"
                if elapsed >= frozen_period
                    rep_state = "move_to_A";
                end
        end
    end
    function set_freeze(val)
        frozen_period = val;
    end


    function onExit()
        try
            for i = 1:6
                mc.disable(motors{i});
            end
        catch
        end
        onClose();
    end

    function onClose()
        stop(t); delete(t);
        stop(rep_timer); delete(rep_timer);
        if ~isempty(mc) && isvalid(mc.serial_)
            delete(mc.serial_);
        end
        delete(fig);
    end
end

