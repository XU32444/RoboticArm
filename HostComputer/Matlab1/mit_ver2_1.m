% Robotic Arm Control UI in MATLAB for 6-DOF Arm (full roomy version)
function robotic_arm_ui

%% ────────────── 1.  Window & top-level grids ──────────────────────
fig = uifigure('Name','6-DOF Robotic Arm UI', ...
               'Position',[100 100 1500 800]);

rootGL = uigridlayout(fig,[1 2], ...
            'ColumnWidth',{'1x',350}, ... % left stretches, right fixed
            'RowHeight',{'1x'});

% LEFT column: scrollable panel that holds six motor rows
scrollP = uipanel(rootGL,'Scrollable','on');
motorGL = uigridlayout(scrollP,[6 1], ...
            'RowHeight',repmat({120},1,6));

% RIGHT column: repetition panel (top) + status box (bottom)
rightGL = uigridlayout(rootGL,[2 1], ...
            'RowHeight',{340,'1x'});

rep_panel = uipanel(rightGL,'Title','Repetition');
statusBox = uitextarea(rightGL,'Editable','off', ...
                       'FontName','Consolas','FontSize',11);

%% ────────────── 2.  Global state / hardware objects ───────────────
motors = cell(1,6);
sliders  = gobjects(1,6);  edits    = gobjects(1,6);
kp_fields = gobjects(1,6); kd_fields = gobjects(1,6);
v_fields  = gobjects(1,6); tau_fields = gobjects(1,6);

target_positions = zeros(1,6);      % live jog target (rad)
kp_vals   = ones (1,6);             % KP per joint
kd_vals   = ones (1,6);             % KD per joint
vdes_vals = zeros(1,6);             % desired vel
tauff_vals= zeros(1,6);             % feed-forward τ

% repetition
p_init_vals  = zeros(1,6);          % Pi (rad)
p_final_vals = zeros(1,6);          % Pf (rad)
frozen_period = 1.0;
rep_active = false; rep_paused = false;
rep_state = "idle"; rep_next_time = 0; rep_timer = [];

% DM_CAN controller
mc = [];

%% ────────────── 3.  Hardware init (try/catch) ─────────────────────
try
    addpath('.\DM_CAN\');                          % adjust if needed
    motor_types = [DM_Motor_Type.DM4340,DM_Motor_Type.DM4340,...
                   DM_Motor_Type.DM4340,DM_Motor_Type.DM4310,...
                   DM_Motor_Type.DM4310,DM_Motor_Type.DM4310];
    slave_ids  = [1 2 3 4 5 6];
    master_ids = [17 18 19 20 21 22];              % 0x11…0x16

    for i = 1:6
        motors{i} = Motor(motor_types(i), slave_ids(i), master_ids(i));
    end

    mc = MotorControl('COM4',921600);

    for i = 1:6
        mc.addMotor(motors{i});
        mc.switchControlMode(motors{i},Control_Type.MIT);
        mc.enable(motors{i});
    end
catch ex
    uialert(fig,ex.message,'Initialization Error');
    return;
end

%% ────────────── 4.  Build LEFT motor rows ─────────────────────────
for i = 1:6
    rowP  = uipanel(motorGL);                       % one panel per joint
    rowGL = uigridlayout(rowP,[1 16], ...
             'ColumnWidth',{70,330,60,60,25,45,25,45,...
                            35,60,35,60,35,60,35,60});

    % Target label
    uilabel(rowGL,'Text',sprintf('M%d Target°',i));

    % Slider & numeric for target
    sliders(i) = uislider(rowGL,'Limits',[-720 720], ...
                 'ValueChangedFcn',@(s,ev) slider_cb(i,ev.Value));
    edits(i)   = uieditfield(rowGL,'numeric','Limits',[-720 720], ...
                 'ValueChangedFcn',@(e,ev) edit_cb(i,ev.Value));

    % Zero button
    uibutton(rowGL,'Text','Zero', ...
             'ButtonPushedFcn',@(b,~) set_zero(i));

    % KP / KD
    uilabel(rowGL,'Text','KP');
kp_fields(i) = uieditfield(rowGL,'numeric','Limits',[0 500], ...
                 'ValueChangedFcn',@(src,~) set_kp(i,src.Value));

    uilabel(rowGL,'Text','KD');
kd_fields(i) = uieditfield(rowGL,'numeric','Limits',[0 5], ...
                 'ValueChangedFcn',@(src,~) set_kd(i, src.Value));

    % Vdes / τ_ff
    uilabel(rowGL,'Text','Vd');
v_fields(i)  = uieditfield(rowGL,'numeric','Limits',[-100 100], ...
                 'ValueChangedFcn',@(src,~) set_vdes(i,src.Value));

    uilabel(rowGL,'Text','τff');
tau_fields(i)= uieditfield(rowGL,'numeric','Limits',[-5 5], ...
                 'ValueChangedFcn',@(src,~) set_tauff(i,src.Value));
end

%% setting up
function set_kd(idx,val),    kd_vals(idx)   = val; end
function set_kp(idx,val),    kp_vals(idx)   = val; end
function set_vdes(idx,val),  vdes_vals(idx) = val; end
function set_tauff(idx,val), tauff_vals(idx)= val; end

function set_freeze(val)
    frozen_period = val;
end
function set_pinit(idx,deg)
    p_init_vals(idx) = deg2rad(deg);
end

function set_pfinal(idx,deg)
    p_final_vals(idx) = deg2rad(deg);
end


%% ────────────── 5.  Build RIGHT repetition panel ──────────────────
repGL = uigridlayout(rep_panel,[9 4], ...
          'ColumnWidth',{70,60,60,'1x'}, ...
          'RowHeight',{26 26 26 26 26 26 26 26 40});

% Row 1: frozen + buttons
uilabel(repGL,'Text','Frozen (s):');
% ----- inside the repetition-panel code -----
uieditfield(repGL,'numeric','Value',frozen_period,'Limits',[0 10], ...
            'ValueChangedFcn',@(src,~) set_freeze(src.Value));  % <-- use setter

uibutton(repGL,'Text','Start','ButtonPushedFcn',@(b,~) start_rep);
uibutton(repGL,'Text','Pause','ButtonPushedFcn',@(b,~) pause_rep);
uibutton(repGL,'Text','Reset','ButtonPushedFcn',@(b,~) reset_rep);

% Headers
uilabel(repGL,'Text','Pi°','Layout',[2 2]);
uilabel(repGL,'Text','Pf°','Layout',[2 3]);

% Six rows of Pi/Pf
for i = 1:6
    r = i+2;
    uilabel(repGL,'Text',sprintf('M%d',i),'Layout',[r 1]);

    pi = uieditfield(repGL,'numeric','Limits',[-720 720], ...
           'ValueChangedFcn',@(src,~) set_pinit(i,src.Value));
    pi.Layout.Row    = r;
    pi.Layout.Column = 2;
    
    % Pf field
    pf = uieditfield(repGL,'numeric','Limits',[-720 720], ...
           'ValueChangedFcn',@(src,~) set_pfinal(i,src.Value));
    pf.Layout.Row    = r;
    pf.Layout.Column = 3;
end

%% ────────────── 6.  Bottom buttons (send / exit) ──────────────────
buttonGL = uigridlayout(fig,[1 2],'ColumnWidth',{'1x','1x'}, ...
                        'RowHeight',{34},'Padding',[0 10 0 10]);
buttonGL.Layout.Row = 2; buttonGL.Layout.Column = 1:2;

uibutton(buttonGL,'Text','Send Command', ...
         'ButtonPushedFcn',@(b,~) send_cmd());
uibutton(buttonGL,'Text','Exit', ...
         'ButtonPushedFcn',@(b,~) onExit());

%% ────────────── 7.  Timers ────────────────────────────────────────
statusTimer = timer('ExecutionMode','fixedRate','Period',0.1, ...
                    'TimerFcn',@refresh_status);
start(statusTimer);

rep_timer = timer('ExecutionMode','fixedRate','Period',0.1, ...
                  'TimerFcn',@rep_step);

fig.CloseRequestFcn = @(~,~) onClose();

%% ────────────── 8.  Callbacks & helpers ───────────────────────────
    function slider_cb(idx,deg)
        edits(idx).Value = deg;
        target_positions(idx) = deg2rad(deg);
    end

    function edit_cb(idx,deg)
        sliders(idx).Value = deg;
        target_positions(idx) = deg2rad(deg);
    end

    function send_cmd
        for k = 1:6
            mc.controlMIT(motors{k}, ...
                kp_vals(k), kd_vals(k), ...
                target_positions(k), vdes_vals(k), tauff_vals(k));
        end
    end

    function set_zero(idx)
        try
            mc.set_zero_position(motors{idx});
            sliders(idx).Value = 0; edits(idx).Value = 0;
            target_positions(idx) = 0;
        catch ex
            uialert(fig,ex.message,sprintf('Zero %d error',idx));
        end
    end

%% ----- repetition state machine -----------------------------------
    function start_rep
        rep_active = true; rep_paused = false;
        rep_state  = "move_to_B";   % first hop to Pf
        rep_next_time = tic;
        start(rep_timer);
    end
    function pause_rep, rep_paused = ~rep_paused; end
    function reset_rep
        rep_active = false; rep_paused = false;
        rep_state = "idle";
        stop(rep_timer);
    end

    function rep_step(~,~)
        if ~rep_active || rep_paused, return; end
        elapsed = toc(rep_next_time);
        switch rep_state
            case "move_to_A"
                for k = 1:6
                    mc.controlMIT(motors{k},kp_vals(k),kd_vals(k), ...
                        p_init_vals(k),vdes_vals(k),tauff_vals(k));
                end
                rep_state="wait_A"; rep_next_time=tic;
            case "wait_A"
                if elapsed>=frozen_period, rep_state="move_to_B"; end
            case "move_to_B"
                for k = 1:6
                    mc.controlMIT(motors{k},kp_vals(k),kd_vals(k), ...
                        p_final_vals(k),vdes_vals(k),tauff_vals(k));
                end
                rep_state="wait_B"; rep_next_time=tic;
            case "wait_B"
                if elapsed>=frozen_period, rep_state="move_to_A"; end
        end
    end

%% ----- periodic status -------------------------------------------
    function refresh_status(~,~)
        try
            lines = cell(6,1);
            for k = 1:6
                mc.refresh_motor_status(motors{k});
                p  = rad2deg(motors{k}.getPosition());
                v  = motors{k}.getVelocity();
                tq = motors{k}.getTorque();
                lines{k} = sprintf(['M%d  P=%6.1f°  V=%6.2f  τ=%5.2f ' ...
                                    'KP=%4.1f KD=%4.2f  Vd=%6.1f  τff=%5.2f'], ...
                                    k,p,v,tq,kp_vals(k),kd_vals(k), ...
                                    vdes_vals(k),tauff_vals(k));
            end
            statusBox.Value = lines;
        catch er
            statusBox.Value = {['Status error: ' er.message]};
        end
    end

%% ----- exit / cleanup --------------------------------------------
    function onExit
        try
            for k = 1:6, mc.disable(motors{k}); end
        catch, end
        onClose();
    end

    function onClose
        stop(statusTimer); delete(statusTimer);
        stop(rep_timer);   delete(rep_timer);
        if ~isempty(mc) && isvalid(mc.serial_), delete(mc.serial_); end
        delete(fig);
    end
end
