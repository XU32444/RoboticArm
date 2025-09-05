% Robotic Arm Control UI in MATLAB for 6-DOF Arm using MIT Mode
% not sure if kp kd is working, the frozen time isnt working neither
function robotic_arm_ui
    % Create UI figure
    fig = uifigure('Name','6-DOF Robotic Arm UI','Position',[100 100 1200 600]);

    % Global storage for motors
    motors = cell(1, 6);
    mc = [];  % MotorControl object
    target_positions = zeros(1, 6); % radians
    sliders = gobjects(1,6);
    edits = gobjects(1,6);
    kp = 1; kd = 1.0; % default MIT gains

    % Repetition test state
    rep_start_vals = zeros(1,6);
    rep_end_vals = zeros(1,6);
    rep_active = false;
    rep_paused = false;
    rep_state = "idle";
    rep_timer = [];
    frozen_period = 5;
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
        uilabel(fig,'Text',sprintf('Motor %d Target (°)',i), 'Position',[50 y_offset 120 22]);
        sliders(i) = uislider(fig, 'Position',[150 y_offset+10 400 3], 'Limits',[-720 720], 'Value', 0, ...
            'ValueChangedFcn', @(sld,event) slider_update_callback(i, event.Value));
        edits(i) = uieditfield(fig, 'numeric', 'Position',[570 y_offset 60 22], 'Limits', [-720 720], 'Value', 0, ...
            'ValueChangedFcn', @(edt,event) edit_update_callback(i, event.Value));
        uibutton(fig, 'Text','Set Zero', 'Position',[640 y_offset 80 22], 'ButtonPushedFcn', @(btn,event) set_zero(i));
    end

    uibutton(fig, 'Text','Send Command','Position',[350 50 120 30], 'ButtonPushedFcn', @(btn,event) send_command());
    uibutton(fig, 'Text','Exit','Position',[500 50 80 30], 'ButtonPushedFcn', @(btn,event) onExit());

    posText = uitextarea(fig, 'Position',[980 200 200 360], 'Editable','off');

    rep_panel = uipanel(fig, 'Title', 'Repetition', 'Position', [750 50 430 130]);
    uilabel(rep_panel, 'Text', 'Frozen Period (s):', 'Position', [10 90 110 22]);
    uieditfield(rep_panel, 'numeric', 'Value', 1.0, 'Position', [130 90 60 22], ...
        'ValueChangedFcn', @(src,event) set_frozen_period(src.Value));

    uilabel(rep_panel, 'Text', 'KP:', 'Position', [200 90 30 22]);
    uieditfield(rep_panel, 'numeric', 'Value', kp, 'Position', [230 90 50 22], 'ValueChangedFcn', @(src,event) set_frozen_period(src.Value));
    uilabel(rep_panel, 'Text', 'KD:', 'Position', [290 90 30 22]);
    uieditfield(rep_panel, 'numeric', 'Value', kd, 'Position', [320 90 50 22], 'ValueChangedFcn', @(src,event) set_frozen_period(src.Value));

    for i = 1:6
        uilabel(rep_panel, 'Text', sprintf('M%d Start/End (°)', i), 'Position', [10 90-i*20 100 22]);
        uieditfield(rep_panel, 'numeric', 'Position', [120 90-i*20 45 22], 'Limits', [-720 720], 'ValueChangedFcn', @(src,event) set_rep_val(i,1,src.Value));
        uieditfield(rep_panel, 'numeric', 'Position', [180 90-i*20 45 22], 'Limits', [-720 720], 'ValueChangedFcn', @(src,event) set_rep_val(i,2,src.Value));
    end

    uibutton(rep_panel, 'Text', 'Start', 'Position', [380 90 40 22], 'ButtonPushedFcn', @(btn,event) start_repetition());
    uibutton(rep_panel, 'Text', 'Pause', 'Position', [380 60 40 22], 'ButtonPushedFcn', @(btn,event) pause_repetition());
    uibutton(rep_panel, 'Text', 'Reset', 'Position', [380 30 40 22], 'ButtonPushedFcn', @(btn,event) reset_repetition());

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
            mc.controlMIT(motors{i}, kp, kd, target_positions(i), 0, 0);
        end
    end

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
            statusStr = "";
            for i = 1:6
                mc.refresh_motor_status(motors{i});
                pos = motors{i}.getPosition();
                vel = motors{i}.getVelocity();
                tau = motors{i}.getTorque();
                statusStr = statusStr + sprintf('M%d: P=%.1f° V=%.2f, T=%.2f\n',i,rad2deg(pos),vel,tau);
            end
            posText.Value = splitlines(statusStr);
        catch
            posText.Value = {"Error Reading Motor"};
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
                    mc.controlMIT(motors{i}, kp, kd, rep_start_vals(i), 0, 0);
                end
                rep_state = "wait_A";
                rep_next_time = tic;
            case "wait_A"
                if elapsed >= frozen_period
                    rep_state = "move_to_B";
                end
            case "move_to_B"
                for i = 1:6
                    mc.controlMIT(motors{i}, kp, kd, rep_end_vals(i), 0, 0);
                end
                rep_state = "wait_B";
                rep_next_time = tic;
            case "wait_B"
                if elapsed >= frozen_period
                    rep_state = "move_to_A";
                end
        end
    end
    function set_frozen_period(val)
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