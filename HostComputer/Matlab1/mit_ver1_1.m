% Robotic Arm Control UI in MATLAB for 6‑DOF Arm using MIT Mode (fixed)
% ----------------------------------------------------------------------
% - No direct assignments inside anonymous functions (avoids "Unsupported '='")
% - Helper setters for frozen_period, kp, kd
% - Removed stray extra "end" after send_command
% ----------------------------------------------------------------------

function mit_ver1_1
    %% UI Figure ---------------------------------------------------------
    fig = uifigure('Name','6‑DOF Robotic Arm UI','Position',[100 100 1200 600]);

    %% Globals / State ---------------------------------------------------
    motors            = cell(1,6);
    mc                = [];           % MotorControl object
    target_positions  = zeros(1,6);   % rad
    sliders           = gobjects(1,6);
    edits             = gobjects(1,6);

    kp = 40;   % default MIT stiffness
    kd = 1.0;  % default MIT damping

    % Repetition‑test state
    rep_start_vals = zeros(1,6);
    rep_end_vals   = zeros(1,6);
    rep_active     = false;
    rep_paused     = false;
    rep_state      = "idle";  % FSM states
    frozen_period  = 1.0;      % seconds to hold at endpoints
    rep_next_time  = 0;        % tic reference

    %% --- Hardware Init -------------------------------------------------
    try
        addpath('.\DM_CAN');   % adjust if needed

        motor_types = [DM_Motor_Type.DM4340, DM_Motor_Type.DM4340, DM_Motor_Type.DM4340, ...
                       DM_Motor_Type.DM4310, DM_Motor_Type.DM4310, DM_Motor_Type.DM4310];
        slave_ids   = [0x01 0x02 0x03 0x04 0x05 0x06];
        master_ids  = [0x11 0x12 0x13 0x14 0x15 0x16];

        for i = 1:6
            motors{i} = Motor(motor_types(i), slave_ids(i), master_ids(i));
        end

        mc = MotorControl('COM4',921600);   % <‑‑ change COM if needed
        for i = 1:6
            mc.addMotor(motors{i});
            mc.switchControlMode(motors{i}, Control_Type.MIT);
            mc.enable(motors{i});
        end
    catch e
        uialert(fig,e.message,'Initialization Error');
        return;
    end

    %% --- Per‑motor controls -------------------------------------------
    for i = 1:6
        y = 520 - (i-1)*70;
        uilabel(fig,'Text',sprintf('Motor %d Target (°)',i),'Position',[50 y 120 22]);

        sliders(i) = uislider(fig,'Position',[150 y+10 400 3],'Limits',[-720 720], ...
            'ValueChangedFcn',@(s,e) set_target_deg(i,e.Value));

        edits(i)   = uieditfield(fig,'numeric','Limits',[-720 720],'Position',[570 y 60 22], ...
            'ValueChangedFcn',@(e,ev) set_target_deg(i,e.Value));

        uibutton(fig,'Text','Set Zero','Position',[640 y 80 22], ...
            'ButtonPushedFcn',@(b,e) set_zero(i));
    end

    %% --- Top‑level buttons -------------------------------------------
    uibutton(fig,'Text','Send Command','Position',[350 50 120 30], ...
        'ButtonPushedFcn',@(b,e) send_command());
    uibutton(fig,'Text','Exit','Position',[500 50 80 30], ...
        'ButtonPushedFcn',@(b,e) onExit());

    %% --- Status box ----------------------------------------------------
    posText = uitextarea(fig,'Position',[980 200 200 360],'Editable','off');

    %% --- Repetition panel ---------------------------------------------
    rep_panel = uipanel(fig,'Title','Repetition','Position',[750 50 430 130]);

    uilabel(rep_panel,'Text','Frozen Period (s):','Position',[10 90 110 22]);
    uieditfield(rep_panel,'numeric','Value',frozen_period,'Position',[130 90 60 22], ...
        'ValueChangedFcn',@(e,ev) set_frozen_period(ev.Value));

    uilabel(rep_panel,'Text','KP:','Position',[200 90 30 22]);
    uieditfield(rep_panel,'numeric','Value',kp,'Position',[230 90 50 22], ...
        'ValueChangedFcn',@(e,ev) set_kp(ev.Value));

    uilabel(rep_panel,'Text','KD:','Position',[290 90 30 22]);
    uieditfield(rep_panel,'numeric','Value',kd,'Position',[320 90 50 22], ...
        'ValueChangedFcn',@(e,ev) set_kd(ev.Value));

    for i = 1:6
        uilabel(rep_panel,'Text',sprintf('M%d Start/End (°)',i),'Position',[10 90-i*20 100 22]);
        uieditfield(rep_panel,'numeric','Position',[120 90-i*20 45 22],'Limits',[-720 720], ...
            'ValueChangedFcn',@(e,ev) set_rep_val(i,1,ev.Value));
        uieditfield(rep_panel,'numeric','Position',[180 90-i*20 45 22],'Limits',[-720 720], ...
            'ValueChangedFcn',@(e,ev) set_rep_val(i,2,ev.Value));
    end

    uibutton(rep_panel,'Text','Start','Position',[380 90 40 22],'ButtonPushedFcn',@(b,e) start_repetition());
    uibutton(rep_panel,'Text','Pause','Position',[380 60 40 22],'ButtonPushedFcn',@(b,e) pause_repetition());
    uibutton(rep_panel,'Text','Reset','Position',[380 30 40 22],'ButtonPushedFcn',@(b,e) reset_repetition());

    %% Timers ------------------------------------------------------------
    uiTimer   = timer('ExecutionMode','fixedRate','Period',0.1,'TimerFcn',@refresh_status);
    repTimer  = timer('ExecutionMode','fixedRate','Period',0.1,'TimerFcn',@repetition_step);
    start(uiTimer);

    fig.CloseRequestFcn = @(src,ev) onClose();

    %% ---------------- Nested helper functions ------------------------

    % Update target from slider or text (degrees)
    function set_target_deg(idx,deg)
        sliders(idx).Value = deg;
        edits(idx).Value   = deg;
        target_positions(idx) = deg2rad(deg);
    end

    % Send MIT command to all motors + debug print
    function send_command()
        dbg = "";
        for i = 1:6
            mc.controlMIT(motors{i},kp,kd,target_positions(i),0,0);
            dbg = dbg + sprintf('M%d → KP=%.1f KD=%.1f Pos=%.1f°\n',i,kp,kd,rad2deg(target_positions(i)));
        end
        disp(dbg);
    end

    % Zero a motor
    function set_zero(idx)
        try
            mc.set_zero_position(motors{idx});
            set_target_deg(idx,0);
            uialert(fig,sprintf('Motor %d zeroed.',idx),'Success');
        catch ex
            uialert(fig,ex.message,sprintf('Zero failed M%d',idx));
        end
    end

    % Status refresh
    function refresh_status(~,~)
        out="";
        for i = 1:6
            mc.refresh_motor_status(motors{i});
            out = out + sprintf('M%d: P=%.1f° V=%.2f T=%.2f\n',i,rad2deg(motors{i}.getPosition()), ...
                                motors{i}.getVelocity(),motors{i}.getTorque());
        end
        posText.Value = splitlines(out);
    end

    % ---------------- Repetition logic ----------------
    function set_rep_val(idx,which,deg)
        if which==1
            rep_start_vals(idx)=deg2rad(deg);
        else
            rep_end_vals(idx)=deg2rad(deg);
        end
    end

    function set_frozen_period(val), frozen_period = val; end
    function set_kp(val), kp = val; end
    function set_kd(val), kd = val; end

    function start_repetition()
        rep_active = true; rep_paused=false; rep_state="move_to_B"; rep_next_time=tic; start(repTimer);
    end
    function pause_repetition(), rep_paused = ~rep_paused; end
    function reset_repetition(), rep_active=false; rep_paused=false; rep_state="idle"; stop(repTimer); end

    function repetition_step(~,~)
        if ~rep_active || rep_paused, return; end
        elapsed = toc(rep_next_time);
        threshold = deg2rad(1);
        reached=true;
        switch rep_state
            case "move_to_A"
                for i=1:6, mc.controlMIT(motors{i},kp,kd,rep_start_vals(i),0,0); end
                rep_state = "wait_A"; rep_next_time=tic;
            case "wait_A"
                for i=1:6
                    mc.refresh_motor_status(motors{i});
                    if abs(motors{i}.getPosition()-rep_start_vals(i))>threshold, reached=false; end
                end
                if reached && elapsed>=frozen_period, rep_state="move_to_B"; end
            case "move_to_B"
                for i=1:6, mc.controlMIT
                end
        end
    end
end