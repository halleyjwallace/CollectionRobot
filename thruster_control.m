function thruster_control(port)
%THRUSTER_CONTROL  Slider GUI for the two front/rear thrusters over USB.
%   thruster_control("COM5")
%
%   Talks to relay_motors_current.ino on the Arduino Uno by sending the line
%   "front,rear,relay\n" (e.g. "1550,1500,1"), where 1500 us = stop and
%   relay 1/0 switches thruster power on/off.
%
%   Sliders are throttle in % (500 us = 100 %). Each motor has a forward and
%   reverse "start" trim (us past 1500 where it begins turning) so both
%   thrusters start together. Use "Find start point" to measure the trims,
%   then copy them into the TRIM constants below so they are kept next time.
%
%   STOP button, the space bar, or closing the window sends 1500,1500,0.
%   Close the Arduino IDE Serial Monitor before running (COM5 is exclusive).

if nargin < 1
    port = "COM5";
end

NEUTRAL     = 1500;   % us, stop
MAX_OFFSET  = 150;    % hard limit on every pulse: 1500 +/- 150 us; raise once tested
MAX_PCT     = 30;     % slider range in % throttle
US_PER_PCT  = 5;      % 500 us from neutral = 100 %
SEND_PERIOD = 0.05;   % s between commands (20 Hz)
ARM_TIME    = 5;      % s the ESCs need at neutral after power-on
RAMP_RATE   = 200;    % us/s, fastest allowed speed change (0 = no limit)
FIND_STEP   = 5;      % us per step when finding a start point
FIND_PERIOD = 0.4;    % s per step when finding a start point

% Start trims in us past 1500 (index 1 = front, 2 = rear)
TRIM_FWD = [0 0];
TRIM_REV = [0 0];

MOTOR_NAMES = ["Front", "Rear"];

targetPct = [0 0];              % slider settings
pulse     = [NEUTRAL NEUTRAL];  % pulses actually sent (after trim + ramp)
relay     = 0;
armed     = false;
rampRate  = RAMP_RATE;
trimFwd   = TRIM_FWD;
trimRev   = TRIM_REV;
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
fprintf("Connected to %s. Waiting 7 s for the Uno to restart...\n", port);
pause(7);
flush(s);

% --------------------------------------------------------------------- UI
fig = uifigure("Name", "Thruster Control", "Position", [100 100 640 540]);
g = uigridlayout(fig, [8 3]);
g.RowHeight   = {60, 60, 30, 30, 35, 45, 70, 45};
g.ColumnWidth = {150, '1x', 150};

% Throttle sliders
sliders    = gobjects(1, 2);
valueLabels = gobjects(1, 2);
for i = 1:2
    lbl = uilabel(g, "Text", MOTOR_NAMES(i) + " (D" + (8 + i) + ")", "FontWeight", "bold");
    lbl.Layout.Row = i; lbl.Layout.Column = 1;
    sliders(i) = uislider(g, "Limits", [-MAX_PCT MAX_PCT], "Value", 0, ...
        "MajorTicks", -MAX_PCT:10:MAX_PCT, ...
        "ValueChangingFcn", @(~, evt) setPct(i, evt.Value), ...
        "ValueChangedFcn",  @(src, ~) setPct(i, src.Value));
    sliders(i).Layout.Row = i; sliders(i).Layout.Column = 2;
    valueLabels(i) = uilabel(g, "Text", "");
    valueLabels(i).Layout.Row = i; valueLabels(i).Layout.Column = 3;
end

% Link + ramp
linkBox = uicheckbox(g, "Text", "Link rear to front (same throttle)");
linkBox.Layout.Row = 3; linkBox.Layout.Column = [1 2];
sub = uigridlayout(g, [1 2], "Padding", [0 0 0 0], "ColumnWidth", {'fit', '1x'});
sub.Layout.Row = 3; sub.Layout.Column = 3;
uilabel(sub, "Text", "Ramp us/s");
uieditfield(sub, "numeric", "Limits", [0 5000], "Value", rampRate, ...
    "ValueChangedFcn", @(src, ~) setRamp(src.Value));

% Start trims
sub = uigridlayout(g, [1 8], "Padding", [0 0 0 0], ...
    "ColumnWidth", {'fit', 45, 'fit', 45, 'fit', 45, 'fit', 45});
sub.Layout.Row = 4; sub.Layout.Column = [1 3];
trimFields = gobjects(2, 2);   % (motor, 1 = fwd / 2 = rev)
for i = 1:2
    for d = 1:2
        dirName = ["fwd", "rev"];
        uilabel(sub, "Text", MOTOR_NAMES(i) + " " + dirName(d) + " start");
        trimFields(i, d) = uieditfield(sub, "numeric", "Limits", [0 MAX_OFFSET], ...
            "RoundFractionalValues", "on", ...
            "ValueChangedFcn", @(src, ~) setTrim(i, d, src.Value));
    end
end
trimFields(1, 1).Value = trimFwd(1); trimFields(1, 2).Value = trimRev(1);
trimFields(2, 1).Value = trimFwd(2); trimFields(2, 2).Value = trimRev(2);

% Find start point
findDrop = uidropdown(g, "Items", ["Front forward", "Front reverse", ...
    "Rear forward", "Rear reverse"]);
findDrop.Layout.Row = 5; findDrop.Layout.Column = 1;
btn = uibutton(g, "Text", "Find start point (slow ramp)", ...
    "ButtonPushedFcn", @(~, ~) startFind());
btn.Layout.Row = 5; btn.Layout.Column = 2;
btn = uibutton(g, "Text", "Turning now!", "FontWeight", "bold", ...
    "ButtonPushedFcn", @(~, ~) turningNow());
btn.Layout.Row = 5; btn.Layout.Column = 3;

% Power buttons
btn = uibutton(g, "Text", "Power ON (arm)", "ButtonPushedFcn", @(~, ~) powerOn());
btn.Layout.Row = 6; btn.Layout.Column = 1;
btn = uibutton(g, "Text", "Zero motors", "ButtonPushedFcn", @(~, ~) zeroMotors());
btn.Layout.Row = 6; btn.Layout.Column = 2;
btn = uibutton(g, "Text", "Power OFF", "ButtonPushedFcn", @(~, ~) stopAll());
btn.Layout.Row = 6; btn.Layout.Column = 3;

btn = uibutton(g, "Text", "STOP  (space bar)", "FontSize", 22, ...
    "FontWeight", "bold", "FontColor", "white", ...
    "BackgroundColor", [0.8 0.1 0.1], "ButtonPushedFcn", @(~, ~) stopAll());
btn.Layout.Row = 7; btn.Layout.Column = [1 3];

% Status row
sub = uigridlayout(g, [1 2], "Padding", [0 0 0 0], "ColumnWidth", {25, '1x'});
sub.Layout.Row = 8; sub.Layout.Column = 1;
relayLamp = uilamp(sub, "Color", [0.5 0.5 0.5]);
logBox = uicheckbox(sub, "Text", "Log CSV", "ValueChangedFcn", @(src, ~) setLogging(src.Value));
statusLabel = uilabel(g, "Text", "Power OFF. Press Power ON to arm the ESCs.", "WordWrap", "on");
statusLabel.Layout.Row = 8; statusLabel.Layout.Column = [2 3];

fig.WindowKeyPressFcn = @onKey;
fig.CloseRequestFcn   = @(~, ~) onClose();

setSlidersEnabled(false);
updateLabels();

% ----------------------------------------------------------------- timers
armTimer = timer("ExecutionMode", "singleShot", "StartDelay", ARM_TIME, ...
    "TimerFcn", @(~, ~) finishArming());
findTimer = timer("ExecutionMode", "fixedSpacing", "Period", FIND_PERIOD, ...
    "BusyMode", "drop", "TimerFcn", @(~, ~) findTick());
sendTimer = timer("ExecutionMode", "fixedSpacing", "Period", SEND_PERIOD, ...
    "BusyMode", "drop", "TimerFcn", @(~, ~) stepAndSend());
start(sendTimer);

% ================================================================ helpers
    function stepAndSend()
        % Move the sent pulses toward the slider targets, then send them.
        dt = toc(lastStep);
        lastStep = tic;
        if finding
            desired = [NEUTRAL NEUTRAL];
            desired(findMotor) = findPulse;
            pulse = desired;
        else
            desired = [pctToPulse(1, targetPct(1)), pctToPulse(2, targetPct(2))];
            if rampRate > 0 && relay == 1
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
            writeline(s, sprintf("%d,%d,%d", pulse(1), pulse(2), relay));
            if logFid > 0
                fprintf(logFid, "%.3f,%d,%d,%d,%g,%g\n", toc(logStart), ...
                    pulse(1), pulse(2), relay, targetPct(1), targetPct(2));
            end
            n = s.NumBytesAvailable;
            if n > 0
                replies = strtrim(splitlines(string(read(s, n, "char"))));
                replies(replies == "") = [];
                if ~isempty(replies)
                    relayLamp.Color = lampColor(replies(end));
                end
            end
        catch err
            statusLabel.Text = "Serial error: " + err.message;
        end
    end

    function p = pctToPulse(i, pct)
        % Throttle % -> pulse, skipping each motor's dead zone via its trim.
        if pct > 0
            p = NEUTRAL + trimFwd(i) + pct * US_PER_PCT;
        elseif pct < 0
            p = NEUTRAL - trimRev(i) + pct * US_PER_PCT;
        else
            p = NEUTRAL;
        end
        p = max(NEUTRAL - MAX_OFFSET, min(NEUTRAL + MAX_OFFSET, p));
    end

    function powerOn()
        stopFind();
        targetPct = [0 0];
        syncSliders(0);
        pulse = [NEUTRAL NEUTRAL];
        relay = 1;
        armed = false;
        setSlidersEnabled(false);
        stepAndSend();
        statusLabel.Text = sprintf("Arming: holding 1500 for %d s, wait for ESC beeps...", ARM_TIME);
        stop(armTimer);
        start(armTimer);
    end

    function finishArming()
        if relay == 1
            armed = true;
            setSlidersEnabled(true);
            statusLabel.Text = "Armed. Move sliders slowly. Space bar = STOP.";
        end
    end

    function zeroMotors()
        % Immediate, not ramped
        stopFind();
        targetPct = [0 0];
        syncSliders(0);
        pulse = [NEUTRAL NEUTRAL];
        stepAndSend();
    end

    function stopAll()
        stop(armTimer);
        stopFind();
        armed = false;
        targetPct = [0 0];
        syncSliders(0);
        pulse = [NEUTRAL NEUTRAL];
        relay = 0;
        stepAndSend();
        setSlidersEnabled(false);
        statusLabel.Text = "STOPPED. Relay off. Press Power ON to arm again.";
    end

    function setPct(i, v)
        if finding
            return
        end
        targetPct(i) = round(v);
        if linkBox.Value
            targetPct(3 - i) = targetPct(i);
        end
        syncSliders(i);
        stepAndSend();   % send now instead of waiting for the timer
    end

    function setRamp(v)
        rampRate = v;
    end

    function setTrim(i, d, v)
        if d == 1
            trimFwd(i) = v;
        else
            trimRev(i) = v;
        end
    end

    % ---------------------------------------------------- find start point
    function startFind()
        if ~armed
            statusLabel.Text = "Press Power ON and wait for arming first.";
            return
        end
        choice = findDrop.Value;
        findMotor = 1 + startsWith(choice, "Rear");
        findDir = 1 - 2 * endsWith(choice, "reverse");
        targetPct = [0 0];
        syncSliders(0);
        pulse = [NEUTRAL NEUTRAL];
        findPulse = NEUTRAL;
        finding = true;
        setSlidersEnabled(false);
        statusLabel.Text = "Finding start point for " + choice + ...
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
            findDrop.Value, findPulse);
    end

    function turningNow()
        if ~finding
            return
        end
        offset = abs(findPulse - NEUTRAL);
        dirIdx = 1 + (findDir < 0);
        setTrim(findMotor, dirIdx, offset);
        trimFields(findMotor, dirIdx).Value = offset;
        choice = findDrop.Value;
        stopFind();
        statusLabel.Text = sprintf("%s start = %d us past 1500 (saved as trim).", choice, offset);
    end

    function stopFind()
        if finding
            stop(findTimer);
            finding = false;
            findPulse = NEUTRAL;
            pulse = [NEUTRAL NEUTRAL];
            setSlidersEnabled(armed);
        end
    end

    % ------------------------------------------------------------ logging
    function setLogging(on)
        if on
            folder = fileparts(mfilename("fullpath"));
            name = "thruster_log_" + string(datetime("now", "Format", "yyyyMMdd_HHmmss")) + ".csv";
            logFid = fopen(fullfile(folder, name), "w");
            fprintf(logFid, "t_s,front_us,rear_us,relay,front_pct,rear_pct\n");
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
        for k = 1:2
            valueLabels(k).Text = sprintf("%+d%%  ->  %d us", targetPct(k), pulse(k));
        end
    end

    function syncSliders(skip)
        % Move sliders to match targetPct, except the one being dragged
        for k = 1:2
            if k ~= skip
                sliders(k).Value = targetPct(k);
            end
        end
    end

    function setSlidersEnabled(on)
        for k = 1:2
            sliders(k).Enable = on;
        end
    end

    function c = lampColor(reply)
        if reply == "HIGH"
            c = [0.1 0.8 0.1];
        else
            c = [0.5 0.5 0.5];
        end
    end

    function onKey(~, evt)
        if evt.Key == "space"
            stopAll();
        end
    end

    function onClose()
        stopAll();
        stop(sendTimer);
        % Repeat the stop so it gets through even if one line is lost
        for k = 1:3
            try
                writeline(s, sprintf("%d,%d,%d", NEUTRAL, NEUTRAL, 0));
            catch
            end
            pause(0.05);
        end
        if logFid > 0
            fclose(logFid);
        end
        delete([sendTimer, findTimer, armTimer]);
        delete(s);
        delete(fig);
        fprintf("Thrusters stopped, relay off, %s released.\n", port);
    end
end
