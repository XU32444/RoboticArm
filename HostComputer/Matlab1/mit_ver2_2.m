function robotic_arm_ui
% 6-DOF Robotic-Arm UI  — MIT mode, roomy grid layout (R2020b+)

%% ─── 0. WINDOW & TOP-LEVEL GRID ───────────────────────────────────
fig = uifigure('Name','6-DOF Arm UI','Position',[100 100 1500 800]);

rootGL = uigridlayout(fig,[2 2], ...          % rows: main + buttons
                      'RowHeight',{'1x',40}, ...
                      'ColumnWidth',{'1x',350});

% --- Left column (scrollable motor rows) --------------------------
scrollP = uipanel(rootGL,'Scrollable','on');
scrollP.Layout.Row = 1; scrollP.Layout.Column = 1;
motorGL = uigridlayout(scrollP,[6 1], ...
                       'RowHeight',repmat({120},1,6));

% --- Right column (Repetition + Status) ---------------------------
rightGL = uigridlayout(rootGL,[2 1],...
                       'RowHeight',{340,'1x'});
rightGL.Layout.Row = 1; rightGL.Layout.Column = 2;

rep_panel = uipanel(rightGL,'Title','Repetition');
statusBox = uitextarea(rightGL,'Editable','off',...
                       'FontName','Consolas','FontSize',11);

% --- Bottom-row buttons ------------------------------------------
btnSend = uibutton(rootGL,'Text','Send Command');
btnSend.Layout.Row = 2; btnSend.Layout.Column = 1;

btnExit = uibutton(rootGL,'Text','Exit');
btnExit.Layout.Row = 2; btnExit.Layout.Column = 2;

%% ─── 1. GLOBAL STATE & HARDWARE ──────────────────────────────────
motors = cell(1,6);
% UI handles
sliders = gobjects(1,6); edits = sliders;
kp_fields = sliders; kd_fields = sliders;
v_fields  = sliders; tau_fields = sliders;

% Controller parameters
target_positions = zeros(1,6);      % live jog target (rad)
kp_vals   = ones (1,6);
kd_vals   = ones (1,6);
vdes_vals = zeros(1,6);
tauff_vals= zeros(1,6);

% Repetition
p_init_vals  = zeros(1,6);
p_final_vals = zeros(1,6);
frozen_period = 1.0;
rep_active = false; rep_paused = false;
rep_state = "idle"; rep_next_time = 0;

mc = [];   % MotorControl object

%% ─── 2. HARDWARE INITIALIZATION ──────────────────────────────────
try
    addpath('.\DM_CAN\');
    motor_types = [DM_Motor_Type.DM4340,DM_Motor_Type.DM4340,...
                   DM_Motor_Type.DM4340,DM_Motor_Type.DM4310,...
                   DM_Motor_Type.DM4310,DM_Motor_Type.DM4310];
    slave_ids  = 1:6;
    master_ids = 0x11:0x16;     % 17–22

    for k = 1:6
        motors{k} = Motor(motor_types(k),slave_ids(k),master_ids(k));
    end

    mc = MotorControl('COM4',921600);
    for k = 1:6
        mc.addMotor(motors{k});
        mc.switchControlMode(motors{k},Control_Type.MIT);
        mc.enable(motors{k});
    end
catch ex
    uialert(fig,ex.message,'Init Error'); return;
end

%% ─── 3. BUILD LEFT MOTOR ROWS ────────────────────────────────────
for k = 1:6
    rowP = uipanel(motorGL);
    rowGL = uigridlayout(rowP,[1 16],...
        'ColumnWidth',{70,330,60,60,30,50,30,50,30,60,30,60,30,60,30,60});

    uilabel(rowGL,'Text',sprintf('M%d Target°',k));

    sliders(k) = uislider(rowGL,'Limits',[-720 720],...
        'ValueChangedFcn',@(s,ev) set_target_deg(k,ev.Value));
    edits(k)   = uieditfield(rowGL,'numeric','Limits',[-720 720],...
        'ValueChangedFcn',@(e,ev) set_target_deg(k,ev.Value));

    uibutton(rowGL,'Text','Zero', 'ButtonPushedFcn',@(b,~) do_zero(k));

    uilabel(rowGL,'Text','KP');
    kp_fields(i) = uieditfield(rowGL,'numeric','Limits',[0 500], ...
                 'ValueChangedFcn',@(src,~) set_kp(src,i));

    uilabel(rowGL,'Text','KD');
    kd_fields(k)=uieditfield(rowGL,'numeric','Limits',[0 5],...
        'ValueChangedFcn',@(e,~) kd_vals(k)=e.Value);

    uilabel(rowGL,'Text','Vd');
    v_fields(k)=uieditfield(rowGL,'numeric','Limits',[-100 100],...
        'ValueChangedFcn',@(e,~) vdes_vals(k)=e.Value);

    uilabel(rowGL,'Text','τff');
    tau_fields(k)=uieditfield(rowGL,'numeric','Limits',[-5 5],...
        'ValueChangedFcn',@(e,~) tauff_vals(k)=e.Value);
end
%% setting up

%% ─── 4. BUILD REPETITION PANEL ───────────────────────────────────
repGL = uigridlayout(rep_panel,[9 4],...
          'ColumnWidth',{70,60,60,'1x'},...
          'RowHeight',{26 26 26 26 26 26 26 26 40});

uilabel(repGL,'Text','Frozen (s):');
efFreeze = uieditfield(repGL,'numeric','Value',frozen_period,'Limits',[0 10]);
efFreeze.ValueChangedFcn = @(e,~) set_freeze(e.Value);

btnStart = uibutton(repGL,'Text','Start','ButtonPushedFcn',@(b,~) rep_start());
btnPause = uibutton(repGL,'Text','Pause','ButtonPushedFcn',@(b,~) rep_pause());
btnReset = uibutton(repGL,'Text','Reset','ButtonPushedFcn',@(b,~) rep_reset());

% headers
lblPi = uilabel(repGL,'Text','Pi°'); lblPi.Layout.Row=2; lblPi.Layout.Column=2;
lblPf = uilabel(repGL,'Text','Pf°'); lblPf.Layout.Row=2; lblPf.Layout.Column=3;

for k = 1:6
    r = k+2;
    lab = uilabel(repGL,'Text',sprintf('M%d',k));
    lab.Layout.Row=r; lab.Layout.Column=1;

    pi = uieditfield(repGL,'numeric','Limits',[-720 720], ...
        'ValueChangedFcn',@(src,~) p_init_vals(k)=deg2rad(src.Value));
    pi.Layout.Row=r; pi.Layout.Column=2;

    pf = uieditfield(repGL,'numeric','Limits',[-720 720], ...
        'ValueChangedFcn',@(src,~) p_final_vals(k)=deg2rad(src.Value));
    pf.Layout.Row=r; pf.Layout.Column=3;
end

%% ─── 5. BUTTON CALLBACKS ─────────────────────────────────────────
btnSend.ButtonPushedFcn = @(~,~) send_cmd();
btnExit.ButtonPushedFcn = @(~,~) onExit();

%% ─── 6. TIMERS ───────────────────────────────────────────────────
statusTimer = timer('ExecutionMode','fixedRate','Period',0.1,...
                    'TimerFcn',@refresh_status);
start(statusTimer);

rep_timer = timer('ExecutionMode','fixedRate','Period',0.1,...
                  'TimerFcn',@rep_step);

fig.CloseRequestFcn = @(~,~) onClose();

%% ─── 7. NESTED FUNCTIONS ─────────────────────────────────────────
    function set_target_deg(idx,deg)
        sliders(idx).Value = deg; edits(idx).Value = deg;
        target_positions(idx) = deg2rad(deg);
    end

    function send_cmd
        for j = 1:6
            mc.controlMIT(motors{j},kp_vals(j),kd_vals(j),...
                          target_positions(j),vdes_vals(j),tauff_vals(j));
        end
    end

    function do_zero(idx)
        try
            mc.set_zero_position(motors{idx});
            set_target_deg(idx,0);
        catch ex, uialert(fig,ex.message,'Zero Error'); end
    end

    function set_freeze(val), frozen_period = val; end

%% ----- repetition state machine ----------------------------------
    function rep_start
        rep_active = true; rep_paused = false;
        rep_state="move_to_B"; rep_next_time=tic; start(rep_timer);
    end
    function rep_pause, rep_paused = ~rep_paused; end
    function rep_reset, rep_active=false; rep_paused=false; rep_state="idle"; stop(rep_timer); end

    function rep_step(~,~)
        if ~rep_active || rep_paused, return; end
        if toc(rep_next_time) < frozen_period && startsWith(rep_state,"wait"), return; end
        switch rep_state
            case "move_to_A"
                for j=1:6, mc.controlMIT(motors{j},kp_vals(j),kd_vals(j),...
                        p_init_vals(j),vdes_vals(j),tauff_vals(j)); end
                rep_state="wait_A"; rep_next_time=tic;
            case "wait_A"
                rep_state="move_to_B";
            case "move_to_B"
                for j=1:6, mc.controlMIT(motors{j},kp_vals(j),kd_vals(j),...
                        p_final_vals(j),vdes_vals(j),tauff_vals(j)); end
                rep_state="wait_B"; rep_next_time=tic;
            case "wait_B"
                rep_state="move_to_A";
        end
    end

%% ----- status ----------------------------------------------------
    function refresh_status(~,~)
        try
            lines = cell(6,1);
            for j=1:6
                mc.refresh_motor_status(motors{j});
                p = rad2deg(motors{j}.getPosition());
                v = motors{j}.getVelocity();
                t = motors{j}.getTorque();
                lines{j} = sprintf(['M%d  P=%6.1f°  V=%6.2f  τ=%5.2f ' ...
                                    'KP=%4.1f KD=%4.2f  Vd=%6.1f  τff=%5.2f'],...
                                    j,p,v,t,kp_vals(j),kd_vals(j),...
                                    vdes_vals(j),tauff_vals(j));
            end
            statusBox.Value = lines;
        catch er
            statusBox.Value = {['Status err: ' er.message]};
        end
    end

%% ----- exit / cleanup -------------------------------------------
    function onExit, onClose(); end
    function onClose
        stop(statusTimer); delete(statusTimer);
        stop(rep_timer);   delete(rep_timer);
        try, for j=1:6, mc.disable(motors{j}); end, catch, end
        if ~isempty(mc)&&isvalid(mc.serial_), delete(mc.serial_); end
        delete(fig);
    end
end
