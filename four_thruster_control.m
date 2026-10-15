function four_thruster_control(port)
%FOUR_THRUSTER_CONTROL  GUI for 4 thrusters + impeller pump over USB.
%   four_thruster_control("COM5")
%
%   Talks to four_thrusters.ino on the Arduino Uno. Every 0.05 s it sends
%   "LF,LR,RF,RR,pump\n" (ESC pulses in us, 1500 = stop; pump 1/0).
%   The Arduino stops everything if commands stop for 0.5 s.
%
%   Drive mode:      Forward + Turn sliders, mixed to left/right hulls.
%   Individual mode: one slider per thruster (for direction checks / trims).
%
%   Each thruster has start trims (us past 1500 where it begins turning,
%   above and below 1500) and a Reverse box that flips its direction in
%   software. Copy found values into TRIM_POS / TRIM_NEG / REVERSED below.
%
%   Impeller ON/OFF button (or the i key) switches the pump relay on D7.
%   The lamp and label show the state the Arduino confirms.
%
%   STOP button, the space bar, or closing the window stops all thrusters
%   and turns the impeller off. The thrusters' real power cut is the main switch.
%   Close the Arduino IDE Serial Monitor before running (COM port is exclusive).

if nargin < 1
    port = "COM5";
end

NEUTRAL     = 1500;   % us, stop
MAX_OFFSET  = 150;    % hard limit on every pulse: 1500 +/- 150 us; raise once tested
MAX_PCT     = 30;     % slider range in % throttle
US_PER_PCT  = 5;      % 500 us from neutral = 100 %
SEND_PERIOD = 0.05;   % s between commands (20 Hz, well inside the 0.5 s failsafe)
RAMP_RATE   = 200;    % us/s, fastest allowed speed change (0 = no limit)
FIND_STEP   = 5;      % us per step when finding a start point
FIND_PERIOD = 0.4;    % s per step when finding a start point

% Per thruster, order: left front, left rear, right front, right rear
NAMES    = ["Left front", "Left rear", "Right front", "Right rear"];
PINS     = ["D9", "D10", "D5", "D11"];
TRIM_POS = [0 0 0 0];               % start trim above 1500 (us)
TRIM_NEG = [0 0 0 0];               % start trim below 1500 (us)
REVERSED = [false false false false]; % true = flip this thruster in software

targetPct = [0 0 0 0];              % per-thruster throttle %
pulse     = NEUTRAL * ones(1, 4);   % pulses actually sent (after trim + ramp)
fwdPct    = 0;
turnPct   = 0;
pump      = 0;
enabled   = false;
driveMode = true;
rampRate  = RAMP_RATE;
trimPos   = TRIM_POS;
trimNeg   = TRIM_NEG;
reversed  = REVERSED;
finding   = false;
findMotor = 1;
findDir   = 1;
findPulse = NEUTRAL;
logFid    = -1;
logStart  = tic;
lastStep  = tic;

% ---------------------------------------------------------------- connect
try
    s = serialport(port, 115200, "Timeout", 1);
catch err
    error("Could not open %s. Close the Arduino Serial Monitor and check " + ...
          "the port with serialportlist.\n%s", port, err.message);
end
configureTerminator(s, "LF");
fprintf("Connected to %s. Waiting 3 s for the Uno to restart...\n", port);
pause(3);
flush(s);

% --------------------------------------------------------------------- UI
fig = uifigure("Name", "Four Thruster Control", "Position", [80 60 820 700]);
g = uigridlayout(fig, [8 3]);
g.RowHeight   = {35, 55, 55, 285, 30, 35, 45, 65};
g.ColumnWidth = {150, '1x', 170};

% Mode + enable
modeSwitch = uiswitch(g, "slider", "Items", {'Drive', 'Individual'}, ...
    "ValueChangedFcn", @(src, ~) setMode(src.Value));
modeSwitch.Layout.Row = 1; modeSwitch.Layout.Column = 1;
statusLabel = uilabel(g, "Text", "", "WordWrap", "on");
statusLabel.Layout.Row = 1; statusLabel.Layout.Column = 2;
enableBtn = uibutton(g, "state", "Text", "Enable thrusters", ...
    "ValueChangedFcn", @(src, ~) setEnabled(src.Value));
enableBtn.Layout.Row = 1; enableBtn.Layout.Column = 3;

% Drive sliders
lbl = uilabel(g, "Text", "Forward", "FontWeight", "bold");
lbl.Layout.Row = 2; lbl.Layout.Column = 1;
fwdSlider = uislider(g, "Limits", [-MAX_PCT MAX_PCT], "Value", 0, ...
    "MajorTicks", -MAX_PCT:10:MAX_PCT, ...
    "ValueChangingFcn", @(~, evt) setFwd(evt.Value), ...
    "ValueChangedFcn",  @(src, ~) setFwd(src.Value));
fwdSlider.Layout.Row = 2; fwdSlider.Layout.Column = 2;
fwdValue = uilabel(g, "Text", "");
fwdValue.Layout.Row = 2; fwdValue.Layout.Column = 3;

lbl = uilabel(g, "Text", "Turn (+ = right)", "FontWeight", "bold");
lbl.Layout.Row = 3; lbl.Layout.Column = 1;
turnSlider = uislider(g, "Limits", [-MAX_PCT MAX_PCT], "Value", 0, ...
    "MajorTicks", -MAX_PCT:10:MAX_PCT, ...
    "ValueChangingFcn", @(~, evt) setTurn(evt.Value), ...
    "ValueChangedFcn",  @(src, ~) setTurn(src.Value));
turnSlider.Layout.Row = 3; turnSlider.Layout.Column = 2;
turnValue = uilabel(g, "Text", "");
turnValue.Layout.Row = 3; turnValue.Layout.Column = 3;

% Per-thruster table
tbl = uigridlayout(g, [5 6], "Padding", [0 0 0 0], ...
    "RowHeight", {20, 55, 55, 55, 55}, ...
    "ColumnWidth", {110, '1x', 120, 60, 60, 70});
tbl.Layout.Row = 4; tbl.Layout.Column = [1 3];
headers = ["Thruster", "Throttle %", "Sent", "Trim +", "Trim -", "Reverse"];
for c = 1:6
    uilabel(tbl, "Text", headers(c), "FontWeight", "bold");
end
motorSliders = gobjects(1, 4);
motorValues  = gobjects(1, 4);
trimPosFields = gobjects(1, 4);
trimNegFields = gobjects(1, 4);
for i = 1:4
    uilabel(tbl, "Text", NAMES(i) + " (" + PINS(i) + ")");
    motorSliders(i) = uislider(tbl, "Limits", [-MAX_PCT MAX_PCT], "Value", 0, ...
        "MajorTicks", -MAX_PCT:10:MAX_PCT, ...
        "ValueChangingFcn", @(~, evt) setMotor(i, evt.Value), ...
        "ValueChangedFcn",  @(src, ~) setMotor(i, src.Value));
    motorValues(i) = uilabel(tbl, "Text", "");
    trimPosFields(i) = uieditfield(tbl, "numeric", "Limits", [0 MAX_OFFSET], ...
        "RoundFractionalValues", "on", "Value", trimPos(i), ...
        "ValueChangedFcn", @(src, ~) setTrim(i, 1, src.Value));
    trimNegFields(i) = uieditfield(tbl, "numeric", "Limits", [0 MAX_OFFSET], ...
        "RoundFractionalValues", "on", "Value", trimNeg(i), ...
        "ValueChangedFcn", @(src, ~) setTrim(i, -1, src.Value));
    uicheckbox(tbl, "Text", "", "Value", reversed(i), ...
        "ValueChangedFcn", @(src, ~) setReversed(i, src.Value));
end

% Ramp + logging
sub = uigridlayout(g, [1 2], "Padding", [0 0 0 0], "ColumnWidth", {'fit', '1x'});
sub.Layout.Row = 5; sub.Layout.Column = 1;
uilabel(sub, "Text", "Ramp us/s");
uieditfield(sub, "numeric", "Limits", [0 5000], "Value", rampRate, ...
    "ValueChangedFcn", @(src, ~) setRamp(src.Value));
logBox = uicheckbox(g, "Text", "Log CSV", "ValueChangedFcn", @(src, ~) setLogging(src.Value));
logBox.Layout.Row = 5; logBox.Layout.Column = 3;

% Find start point (physical pulse direction: above or below 1500)
findItems = strings(1, 8);
for i = 1:4
    findItems(2*i - 1) = NAMES(i) + "  above 1500";
    findItems(2*i)     = NAMES(i) + "  below 1500";
end
findDrop = uidropdown(g, "Items", findItems, "ItemsData", 1:8);
findDrop.Layout.Row = 6; findDrop.Layout.Column = 1;
btn = uibutton(g, "Text", "Find start point (slow ramp)", ...
    "ButtonPushedFcn", @(~, ~) startFind());
btn.Layout.Row = 6; btn.Layout.Column = 2;
btn = uibutton(g, "Text", "Turning now!", "FontWeight", "bold", ...
    "ButtonPushedFcn", @(~, ~) turningNow());
btn.Layout.Row = 6; btn.Layout.Column = 3;

% Impeller (pump relay on D7) + zero
sub = uigridlayout(g, [1 2], "Padding", [0 0 0 0], "ColumnWidth", {25, '1x'});
sub.Layout.Row = 7; sub.Layout.Column = 1;
pumpLamp = uilamp(sub, "Color", [0.5 0.5 0.5]);
pumpBtn = uibutton(sub, "state", "Text", "Impeller ON", "FontWeight", "bold", ...
    "ValueChangedFcn", @(src, ~) setPumpBtn(src.Value));
btn = uibutton(g, "Text", "Zero thrusters", "ButtonPushedFcn", @(~, ~) zeroThrusters());
btn.Layout.Row = 7; btn.Layout.Column = 2;
impellerLabel = uilabel(g, "Text", "Impeller (relay D7): OFF   [key: i]");
impellerLabel.Layout.Row = 7; impellerLabel.Layout.Column = 3;

btn = uibutton(g, "Text", "STOP  (space bar)", "FontSize", 22, ...
    "FontWeight", "bold", "FontColor", "white", ...
    "BackgroundColor", [0.8 0.1 0.1], "ButtonPushedFcn", @(~, ~) stopAll());
btn.Layout.Row = 8; btn.Layout.Column = [1 3];

fig.WindowKeyPressFcn = @onKey;
fig.CloseRequestFcn   = @(~, ~) onClose();

updateEnables();
updateLabels();
statusLabel.Text = "Main switch ON (ESCs arm at 1500), then press Enable thrusters.";

% ----------------------------------------------------------------- timers
findTimer = timer("ExecutionMode", "fixedSpacing", "Period", FIND_PERIOD, ...
    "BusyMode", "drop", "TimerFcn", @(~, ~) findTick());
sendTimer = timer("ExecutionMode", "fixedSpacing", "Period", SEND_PERIOD, ...
    "BusyMode", "drop", "TimerFcn", @(~, ~) stepAndSend());
start(sendTimer);

% ================================================================ helpers
    function stepAndSend()
        % Move the sent pulses toward the targets, then send them.
        dt = toc(lastStep);
        lastStep = tic;
        if finding
            desired = NEUTRAL * ones(1, 4);
            desired(findMotor) = findPulse;
            pulse = desired;
        elseif ~enabled
            pulse = NEUTRAL * ones(1, 4);
        else
            desired = arrayfun(@pctToPulse, 1:4, targetPct);
            if rampRate > 0
                maxStep = max(1, rampRate * dt);
                pulse = pulse + max(-maxStep, min(maxStep, desired - pulse));
            else
                pulse = desired;
            end
        end
        pulse = round(pulse);
        sendLine();
        updateLabels();
    end

    function sendLine()
        try
            writeline(s, sprintf("%d,%d,%d,%d,%d", [pulse pump]));
            if logFid > 0
                fprintf(logFid, "%.3f,%d,%d,%d,%d,%d,%g,%g\n", toc(logStart), ...
                    pulse, pump, fwdPct, turnPct);
            end
            n = s.NumBytesAvailable;
            if n > 0
                handleReplies(strtrim(splitlines(string(read(s, n, "char")))));
            end
        catch err
            statusLabel.Text = "Serial error: " + err.message;
        end
    end

    function handleReplies(replies)
        for r = replies'
            switch r
                case "PUMP ON"
                    pumpLamp.Color = [0.1 0.8 0.1];
                    impellerLabel.Text = "Impeller (relay D7): ON   [key: i]";
                case "PUMP OFF"
                    pumpLamp.Color = [0.5 0.5 0.5];
                    impellerLabel.Text = "Impeller (relay D7): OFF   [key: i]";
                case "FAILSAFE"
                    statusLabel.Text = "Arduino FAILSAFE: commands stopped for 0.5 s.";
            end
        end
    end

    function p = pctToPulse(i, pct)
        % Throttle % -> pulse: apply Reverse, then skip the dead zone via trims.
        if reversed(i)
            pct = -pct;
        end
        if pct > 0
            p = NEUTRAL + trimPos(i) + pct * US_PER_PCT;
        elseif pct < 0
            p = NEUTRAL - trimNeg(i) + pct * US_PER_PCT;
        else
            p = NEUTRAL;
        end
        p = max(NEUTRAL - MAX_OFFSET, min(NEUTRAL + MAX_OFFSET, p));
    end

    % ------------------------------------------------------------ commands
    function setFwd(v)
        fwdPct = round(v);
        mixDrive();
    end

    function setTurn(v)
        turnPct = round(v);
        mixDrive();
    end

    function mixDrive()
        % Left hull = forward + turn, right hull = forward - turn
        left  = max(-MAX_PCT, min(MAX_PCT, fwdPct + turnPct));
        right = max(-MAX_PCT, min(MAX_PCT, fwdPct - turnPct));
        targetPct = [left left right right];
        syncMotorSliders();
        stepAndSend();   % send now instead of waiting for the timer
    end

    function setMotor(i, v)
        if driveMode || finding
            return
        end
        targetPct(i) = round(v);
        stepAndSend();
    end

    function setMode(value)
        driveMode = string(value) == "Drive";
        zeroThrusters();
        updateEnables();
    end

    function setEnabled(on)
        zeroThrusters();
        enabled = on;
        if on
            enableBtn.Text = "Disable thrusters";
            statusLabel.Text = "Thrusters enabled. Move sliders slowly. Space bar = STOP.";
        else
            enableBtn.Text = "Enable thrusters";
            statusLabel.Text = "Thrusters disabled (all at 1500).";
        end
        updateEnables();
    end

    function setPumpBtn(on)
        % Impeller runs independently of the thrusters (no Enable needed)
        pump = double(on);
        if on
            pumpBtn.Text = "Impeller OFF";
        else
            pumpBtn.Text = "Impeller ON";
        end
        stepAndSend();
    end

    function zeroThrusters()
        % Immediate, not ramped
        stopFind();
        fwdPct = 0;
        turnPct = 0;
        targetPct = [0 0 0 0];
        fwdSlider.Value = 0;
        turnSlider.Value = 0;
        syncMotorSliders();
        pulse = NEUTRAL * ones(1, 4);
        stepAndSend();
    end

    function stopAll()
        zeroThrusters();
        enabled = false;
        enableBtn.Value = false;
        enableBtn.Text = "Enable thrusters";
        pumpBtn.Value = false;
        pumpBtn.Text = "Impeller ON";
        pump = 0;
        stepAndSend();
        updateEnables();
        statusLabel.Text = "STOPPED: thrusters at 1500, impeller off. Main switch cuts thruster power.";
    end

    function setRamp(v)
        rampRate = v;
    end

    function setTrim(i, dir, v)
        if dir > 0
            trimPos(i) = v;
        else
            trimNeg(i) = v;
        end
    end

    function setReversed(i, on)
        reversed(i) = on;
    end

    % ---------------------------------------------------- find start point
    function startFind()
        if ~enabled
            statusLabel.Text = "Turn the main switch on and press Enable thrusters first.";
            return
        end
        zeroThrusters();
        choice = findDrop.Value;
        findMotor = ceil(choice / 2);
        findDir = 1 - 2 * (mod(choice, 2) == 0);
        findPulse = NEUTRAL;
        finding = true;
        updateEnables();
        statusLabel.Text = "Finding start for " + findItems(choice) + ...
            ". Press 'Turning now!' the moment it turns steadily.";
        stop(findTimer);
        start(findTimer);
    end

    function findTick()
        findPulse = findPulse + findDir * FIND_STEP;
        if abs(findPulse - NEUTRAL) > MAX_OFFSET
            stopFind();
            statusLabel.Text = sprintf("No movement up to %d us past neutral. " + ...
                "Check power/wiring, or raise MAX_OFFSET.", MAX_OFFSET);
            return
        end
        statusLabel.Text = sprintf("Finding: %s at %d us ... press 'Turning now!'", ...
            findItems(findDrop.Value), findPulse);
    end

    function turningNow()
        if ~finding
            return
        end
        offset = abs(findPulse - NEUTRAL);
        if findDir > 0
            trimPos(findMotor) = offset;
            trimPosFields(findMotor).Value = offset;
        else
            trimNeg(findMotor) = offset;
            trimNegFields(findMotor).Value = offset;
        end
        label = findItems(findDrop.Value);
        stopFind();
        statusLabel.Text = sprintf("%s: starts %d us from 1500 (saved as trim).", label, offset);
    end

    function stopFind()
        if finding
            stop(findTimer);
            finding = false;
            findPulse = NEUTRAL;
            pulse = NEUTRAL * ones(1, 4);
            updateEnables();
        end
    end

    % ------------------------------------------------------------ logging
    function setLogging(on)
        if on
            folder = fileparts(mfilename("fullpath"));
            name = "thruster_log_" + string(datetime("now", "Format", "yyyyMMdd_HHmmss")) + ".csv";
            logFid = fopen(fullfile(folder, name), "w");
            fprintf(logFid, "t_s,LF_us,LR_us,RF_us,RR_us,pump,fwd_pct,turn_pct\n");
            logStart = tic;
            statusLabel.Text = "Logging to " + name;
        elseif logFid > 0
            fclose(logFid);
            logFid = -1;
            statusLabel.Text = "Logging stopped.";
        end
    end

    % ----------------------------------------------------------------- UI
    function updateLabels()
        fwdValue.Text  = sprintf("%+d%%", fwdPct);
        turnValue.Text = sprintf("%+d%%", turnPct);
        for k = 1:4
            motorValues(k).Text = sprintf("%+d%%  ->  %d us", targetPct(k), pulse(k));
        end
    end

    function syncMotorSliders()
        for k = 1:4
            motorSliders(k).Value = targetPct(k);
        end
    end

    function updateEnables()
        driveOn = enabled && driveMode && ~finding;
        indivOn = enabled && ~driveMode && ~finding;
        fwdSlider.Enable  = driveOn;
        turnSlider.Enable = driveOn;
        for k = 1:4
            motorSliders(k).Enable = indivOn;
        end
    end

    function onKey(~, evt)
        if evt.Key == "space"
            stopAll();
        elseif evt.Key == "i"
            pumpBtn.Value = ~pumpBtn.Value;
            setPumpBtn(pumpBtn.Value);
        end
    end

    function onClose()
        stopAll();
        stop(sendTimer);
        % Repeat the stop so it gets through even if one line is lost
        for k = 1:3
            try
                writeline(s, sprintf("%d,%d,%d,%d,%d", NEUTRAL, NEUTRAL, NEUTRAL, NEUTRAL, 0));
            catch
            end
            pause(0.05);
        end
        if logFid > 0
            fclose(logFid);
        end
        delete([sendTimer, findTimer]);
        delete(s);
        delete(fig);
        fprintf("Thrusters stopped, pump off, %s released.\n", port);
    end
end
