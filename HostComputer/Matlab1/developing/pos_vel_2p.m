% Robotic Arm Control UI in MATLAB for 6-DOF Arm

function robotic_arm_ui1
    % Create UI figure
    fig = uifigure('Name','6-DOF Robotic Arm UI','Position',[100 100 1100 600]);

    % Global storage for motors
    motors = cell(1, 6);
    mc = [];  % MotorControl object
    target_positions = zeros(1, 6); % store target positions from sliders (radians)
    sliders = gobjects(1,6);
    edits = gobjects(1,6);

    % Repetition test state
    rep_start_vals = zeros(1,6);
    rep_end_vals = zeros(1,6);
    rep_active = false;
    rep_paused = false;
    rep_state = "idle"; % states: idle, move_to_A, wait_A, move_to_B, wait_B
    rep_timer = [];
    frozen_period = 1.0; % default 1 second
    rep_next_time = 0;

    % Initialize Motor Connection
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
            mc.switchControlMode(motors{i}, Control_Type.POS_VEL);
        end

        for i = 1:6
            mc.enable(motors{i});
        end
    catch e
        uialert(fig, e.message, 'Initialization Error');
        return;
    end

    % Sliders and input boxes to control motors
    for i = 1:6
        y_offset = 520 - (i-1)*70;
        uilabel(fig,'Text',sprintf('Motor %d Target (°)',i), 'Position',[50 y_offset 120 22]);
        sliders(i) = uislider(fig, 'Position',[150 y_offset+10 400 3], 'Limits',[-720 720], 'Value', 0, ...
            'ValueChangedFcn', @(sld,event) slider_update_callback(i, event.Value));
        edits(i) = uieditfield(fig, 'numeric', 'Position',[570 y_offset 60 22], 'Limits', [-720 720], 'Value', 0, ...
            'ValueChangedFcn', @(edt,event) edit_update_callback(i, event.Value));
        uibutton(fig, 'Text','Set Zero', 'Position',[640 y_offset 80 22], 'ButtonPushedFcn', @(btn,event) set_zero(i));
    end

    % Send Command Button
    uibutton(fig, 'Text','Send Command','Position',[350 50 120 30], 'ButtonPushedFcn', @(btn,event) send_command());

    % Exit Button
    uibutton(fig, 'Text','Exit','Position',[500 50 80 30], 'ButtonPushedFcn', @(btn,event) onExit());

    % Status display
    posText = uitextarea(fig, 'Position',[880 200 200 360], 'Editable','off');

    % Repetition Testing UI
    rep_panel = uipanel(fig, 'Title', 'Repetition', 'Position', [750 100 330 170]);     %left bottom width height
    uilabel(rep_panel, 'Text', 'Frozen Period (s):', 'Position', [10 120 110 22]);
    frozen_input = uieditfield(rep_panel, 'numeric', 'Value', 1.0, 'Position', [130 120 60 22], ...
        'ValueChangedFcn', @(src,event) frozen_period_update(src.Value));

    for i = 1:60
        s = 120;
        uilabel(rep_panel, 'Text', sprintf('M%d Start/End (°)', i), 'Position', [10 s-i*20 100 22]);
        uieditfield(rep_panel, 'numeric', 'Position', [120 s-i*20 45 22], 'Limits', [-720 720], 'ValueChangedFcn', @(src,event)set_rep_val(i,1,src.Value));
        uieditfield(rep_panel, 'numeric', 'Position', [180 s-i*20 45 22], 'Limits', [-720 720], 'ValueChangedFcn', @(src,event)set_rep_val(i,2,src.Value));
    end

    uibutton(rep_panel, 'Text', 'Start', 'Position', [240 90 70 22], 'ButtonPushedFcn', @(btn,event) start_repetition());
    uibutton(rep_panel, 'Text', 'Pause', 'Position', [240 60 70 22], 'ButtonPushedFcn', @(btn,event) pause_repetition());
    uibutton(rep_panel, 'Text', 'Reset', 'Position', [240 30 70 22], 'ButtonPushedFcn', @(btn,event) reset_repetition());

    % Refresh Timer
    t = timer('ExecutionMode','fixedRate','Period',0.1,'TimerFcn',@refresh_status);
    start(t);

    rep_timer = timer('ExecutionMode','fixedRate','Period',0.1,'TimerFcn',@repetition_step);

    fig.CloseRequestFcn = @(src, event) onClose();

    function slider_update_callback(motorIndex, value_deg)
        edits(motorIndex).Value = value_deg;
        target_positions(motorIndex) = deg2rad(value_deg);
    end

    function edit_update_callback(motorIndex, value_deg)
        sliders(motorIndex).Value = value_deg;
        target_positions(motorIndex) = deg2rad(value_deg);
    end

    function send_command()
        for i = 1:6
            mc.control_Pos_Vel(motors{i}, target_positions(i), 2);
        end
    end

    function set_zero(motorIndex)
        try
            mc.set_zero_position(motors{motorIndex});
            sliders(motorIndex).Value = 0;
            edits(motorIndex).Value = 0;
            target_positions(motorIndex) = 0;
            uialert(fig, sprintf('Motor %d zero set.', motorIndex), 'Success');
        catch e
            uialert(fig, e.message, sprintf('Set Zero Failed for Motor %d', motorIndex));
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

    function frozen_period_update(val)
        frozen_period = val;
    end

    function set_rep_val(idx, pos, val_deg)
        if pos == 1
            rep_start_vals(idx) = deg2rad(val_deg);
        else
            rep_end_vals(idx) = deg2rad(val_deg);
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
                    mc.control_Pos_Vel(motors{i}, rep_start_vals(i), 2);
                end
                rep_state = "wait_A";
                rep_next_time = tic;

            case "wait_A"
                if elapsed >= frozen_period
                    rep_state = "move_to_B";
                end

            case "move_to_B"
                for i = 1:6
                    mc.control_Pos_Vel(motors{i}, rep_end_vals(i), 2);
                end
                rep_state = "wait_B";
                rep_next_time = tic;

            case "wait_B"
                if elapsed >= frozen_period
                    rep_state = "move_to_A";
                end
        end
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
