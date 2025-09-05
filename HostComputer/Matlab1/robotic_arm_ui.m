% Robotic Arm Control UI in MATLAB for 6-DOF Arm

function robotic_arm_ui
    % Create UI figure
    fig = uifigure('Name','6-DOF Robotic Arm UI','Position',[100 100 900 600]);

    % Global storage for motors
    motors = cell(1, 6);
    mc = [];  % MotorControl object
    target_positions = zeros(1, 6); % store target positions from sliders (radians)

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

    sliders = gobjects(1,6);
    edits = gobjects(1,6);

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
    posText = uitextarea(fig, 'Position',[750 200 120 360], 'Editable','off');

    % Refresh Timer
    t = timer('ExecutionMode','fixedRate','Period',0.1,'TimerFcn',@refresh_status);
    start(t);

    % Cleanup on close
    fig.CloseRequestFcn = @(src, event) onClose();

    % --- Update target position from slider (in degrees) ---
    function slider_update_callback(motorIndex, value_deg)
        edits(motorIndex).Value = value_deg;
        target_positions(motorIndex) = deg2rad(value_deg);
    end

    % --- Update target position from edit box (in degrees) ---
    function edit_update_callback(motorIndex, value_deg)
        sliders(motorIndex).Value = value_deg;
        target_positions(motorIndex) = deg2rad(value_deg);
    end

    % --- Send POS_VEL command when button is pressed ---
    function send_command()
        for i = 1:6
            mc.control_Pos_Vel(motors{i}, target_positions(i), 2);
        end
    end

    % --- Set Zero Position for a given motor ---
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

    % --- Timer function to refresh motor states ---
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

    % --- Exit button handler ---
    function onExit()
        try
            for i = 1:6
                mc.disable(motors{i});
            end
        catch
        end
        onClose();
    end

    % --- Cleanup on Close ---
    function onClose()
        stop(t); delete(t);
        if ~isempty(mc) && isvalid(mc.serial_)
            delete(mc.serial_);
        end
        delete(fig);
    end
end
