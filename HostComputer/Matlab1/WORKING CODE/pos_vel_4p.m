function robotic_arm_fsm_4p
    % UI & state
    fig = uifigure('Name','Robotic Arm 4P FSM','Position',[100 100 1100 600]);
    motors = cell(1,6);
    target_positions = zeros(1,6);
    mc = [];

    % Motor init
    try
        addpath('.\DM_CAN\');
        motor_types = [DM_Motor_Type.DM4340,DM_Motor_Type.DM4340,DM_Motor_Type.DM4340,...
                       DM_Motor_Type.DM4310,DM_Motor_Type.DM4310,DM_Motor_Type.DM4310];
        slave_ids = 1:6; master_ids = 17:22;    % 17:22;
        for i = 1:6
            motors{i} = Motor(motor_types(i), slave_ids(i), master_ids(i));
        end
        mc = MotorControl('COM4', 921600);
        for i = 1:6
            mc.addMotor(motors{i});
            mc.switchControlMode(motors{i}, Control_Type.POS_VEL);
            mc.enable(motors{i});
        end
    catch e
        uialert(fig, e.message, 'Initialization Error');
        return;
    end

    % UI layout
    edits = gobjects(4,6);  % P1~P4, M1~M6
    poses = zeros(4,6);     % radians
    frozen_period = 1.0;
    pose_idx = 1;
    rep_active = false; rep_paused = false;
    state = "idle";
    rep_timer = timer('ExecutionMode','fixedRate','Period',0.1,'TimerFcn',@step_fsm);
    next_time = 0;
    record_index = 1;

    % Pose Input Grid
    panel = uipanel(fig,'Title','4-Point Repetition','Position',[700 100 370 260]);
    uilabel(panel,'Text','Frozen (s):','Position',[10 220 70 22]);
    uieditfield(panel,'numeric','Position',[80 220 60 22],'Value',1.0,...
        'ValueChangedFcn',@(e,~) frozen_period_update(e.Value));

    for p = 1:4
        uilabel(panel,'Text',['P' num2str(p)],'Position',[10 190 - (p-1)*30 25 22]);
        for m = 1:6
            edits(p,m) = uieditfield(panel,'numeric','Position',[40+(m-1)*50,190 - (p-1)*30,45,22],...
                'Limits',[-720 720],'ValueChangedFcn',@(e,~) set_pose(p,m,e.Value));
        end
    end

    uibutton(panel,'Text','Start','Position',[300 190 60 22],'ButtonPushedFcn',@(btn,~) start_repetition());
    uibutton(panel,'Text','Pause','Position',[300 155 60 22],'ButtonPushedFcn',@(btn,~) pause_repetition());
    uibutton(panel,'Text','Reset','Position',[300 120 60 22],'ButtonPushedFcn',@(btn,~) reset_repetition());
    uibutton(panel,'Text','Record Pose','Position',[300 85 60 22],'ButtonPushedFcn',@(btn,~) record_pose());
    uibutton(panel,'Text','Clear','Position',[300 50 60 22],'ButtonPushedFcn',@(btn,~) clear_poses());

    % Exit + Soft Button + Status
    uibutton(fig,'Text','Make Soft','Position',[400 30 80 30],'ButtonPushedFcn',@(btn,~) make_soft());
    uibutton(fig,'Text','Exit','Position',[500 30 80 30],'ButtonPushedFcn',@(btn,~) onExit());
    statusText = uitextarea(fig, 'Position',[50 50 600 100], 'Editable','off');
    t_status = timer('ExecutionMode','fixedRate','Period',0.5,'TimerFcn',@refresh_status);
    start(t_status);

    fig.CloseRequestFcn = @(src,~) onExit();

    %% --------- FSM Step Function -----------
    function step_fsm(~,~)
        if ~rep_active || rep_paused, return; end
        elapsed = toc(next_time);

        switch state
            case "move"
                for i = 1:6
                    mc.control_Pos_Vel(motors{i}, poses(pose_idx,i), 2);
                end
                state = "wait";
                next_time = tic;

            case "wait"
                if elapsed >= frozen_period
                    pose_idx = mod(pose_idx,4)+1;
                    state = "move";
                end
        end
    end

    %% ---------- UI Callbacks -----------
    function set_pose(p,m,deg)
        poses(p,m) = deg2rad(deg);
    end

    function frozen_period_update(val)
        frozen_period = val;
    end

    function start_repetition()
        for i = 1:6
            mc.switchControlMode(motors{i}, Control_Type.POS_VEL);
            pause(0.01);
            mc.enable(motors{i});
        end
        rep_active = true;
        rep_paused = false;
        state = "move";
        pose_idx = 1;
        next_time = tic;
        if strcmp(rep_timer.Running, 'off')
            start(rep_timer);
        end
    end

    function pause_repetition()
        rep_paused = ~rep_paused;
    end

    function reset_repetition()
        rep_active = false;
        rep_paused = false;
        state = "idle";
        stop(rep_timer);
    end

    function record_pose()
        for i = 1:6
            mc.refresh_motor_status(motors{i});
            poses(record_index,i) = motors{i}.getPosition();
            edits(record_index,i).Value = rad2deg(poses(record_index,i));
        end
        record_index = mod(record_index,4)+1;
    end

    function clear_poses()
        poses(:,:) = 0;
        for i = 1:4
            for j = 1:6
                edits(i,j).Value = 0;
            end
        end
        record_index = 1;
    end

    function refresh_status(~,~)
        lines = strings(1,6);
        try
            for i = 1:6
                mc.refresh_motor_status(motors{i});
                pos = rad2deg(motors{i}.getPosition());
                vel = motors{i}.getVelocity();
                tau = motors{i}.getTorque();
                lines(i) = sprintf("M%d: P=%.1f° V=%.2f τ=%.2f", i, pos, vel, tau);
            end
        catch
            lines = "Status error";
        end
        statusText.Value = lines;
    end

    %% ---------- Make Soft -----------
    function make_soft()
        for i = 1:6
            mc.switchControlMode(motors{i}, Control_Type.MIT);
            pause(0.01);
            mc.enable(motors{i});
            mc.controlMIT(motors{i}, 0, 0, motors{i}.getPosition(), 0, 0);
        end
    end

    %% ---------- Exit Cleanup -----------
    function onExit()
        try
            rep_active = false;
            stop(rep_timer); delete(rep_timer);
            stop(t_status); delete(t_status);
            for i = 1:6
                mc.disable(motors{i});
            end
            if isfield(mc,'serial_') && isvalid(mc.serial_)
                try, fclose(mc.serial_); catch, end
                delete(mc.serial_);
            end
            clear mc;
        catch, end
        delete(fig);
    end
end
