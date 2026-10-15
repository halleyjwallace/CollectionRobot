function drive_control(port)
%DRIVE_CONTROL  Arrow-key driving: forward speed + differential-thrust turning.
%   drive_control("COM5")
%
%   Talks to four_thrusters.ino on the Arduino Uno.
%
%   Keys (click an empty part of the window first so it has keyboard focus):
%     Up / Down     forward speed (surge) +/- 5 %
%     hold Left     turn left  (right side pushes harder, left side less)
%     hold Right    turn right (left side pushes harder, right side less)
%     0             ramp smoothly to stop       Space  STOP (instant)
%     i             impeller on/off             m      mark problem
%
%   MIXER (boat version of a quadcopter mixer). All 4 thrusters push along
%   the hulls, so only surge (forward) and yaw (turning) can be controlled:
%       left thrust  = surge thrust + turn thrust     -> LF, LR
%       right thrust = surge thrust - turn thrust     -> RF, RR
%   Thrust grows roughly with throttle^2, and reverse thrust is weaker than
%   forward (REV_RATIO), so the mixing is done in thrust and converted back
%   to throttle. "Turn %" = the thrust one thruster makes at that throttle.
%
%   "Turn step test" applies +turn, 0, -turn, 0 for STEP_TIME s each at the
%   current forward speed. Tick "Record data" to save it for later analysis.
%
%   IMU: the Arduino sends MPU-6050 yaw rate, integrated yaw and acceleration
%   at 20 Hz. The window shows yaw rate (+ = turning right), heading (drifts
%   slowly - re-zero it), and roll/pitch from gravity; all are recorded.

if nargin < 1
    port = "COM5";
end

% ---------------------------------------------------------------- settings
NEUTRAL      = 1500;   % us, stop
MAX_PCT      = 100;    % throttle limit in %
US_PER_PCT   = 5;      % 500 us from neutral = 100 %
MAX_OFFSET   = 500;    % hard limit on every pulse: 1500 +/- 500 us
SEND_PERIOD  = 0.05;   % s between commands (20 Hz, inside the 0.5 s failsafe)
RAMP_UP      = 20;     % %/s when a thruster speeds up   (0 = instant)
RAMP_DOWN    = 40;     % %/s when a thruster slows down  (0 = instant)
SURGE_STEP   = 5;      % % per Up/Down press
MIN_PCT      = 10;     % slowest non-zero throttle; below this the motors stutter
TURN_PCT     = 10;     % default turn strength (%), held Left/Right
TURN_MAX     = 30;     % largest turn strength allowed in the field
STEP_TIME    = 3;      % s per phase of the turn step test

% Model (rough, refine with measurements)
REV_RATIO    = 0.8;    % reverse thrust / forward thrust at the same throttle (calibrate)
MAX_THRUST_N = NaN;    % forward thrust of ONE U1 at 100 % in newtons (from datasheet), NaN = unknown
HULL_SPACING = 0.385;  % m between hull centrelines
THRUST_SPAN  = 0.26;   %#ok<NASGU> m between front and rear thrusters on a hull (for later models)
MASS_KG      = 8;      %#ok<NASGU> kg, approx. 15-20 lb (for later models)

% IMU (MPU-6050). Check on the stand: turn the robot clockwise seen from
% above (a RIGHT turn) - the yaw rate must read positive. If negative, flip.
YAW_SIGN     = -1;     % -1 = MPU mounted flat with the chip facing up, +1 = chip facing down

% Per thruster, order: left front, left rear, right front, right rear
NAMES    = ["LF", "LR", "RF", "RR"];
PINS     = ["D9", "D10", "D5", "D11"];
REVERSED = [false false false false]; % true = flip this thruster's direction
TRIM_POS = [0 0 0 0];                 % start trim above 1500 (us)
TRIM_NEG = [0 0 0 0];                 % start trim below 1500 (us)

% ------------------------------------------------------------------- state
surgePct      = 0;            % commanded forward speed (%)
turnKey       = 0;            % -1 = left held, +1 = right held, 0 = none
turnPct       = TURN_PCT;
straightTrim  = 0;            % constant turn offset (%) to correct drift
targetPct     = [0 0 0 0];    % per-thruster target after mixing
actualPct     = [0 0 0 0];    % per-thruster ramped value actually sent
pulse         = NEUTRAL * ones(1, 4);
pump          = 0;
enabled       = false;
stepIdx       = 0;            % turn step test phase (0 = not running)
stepSeq       = [1 0 -1 0];
stepClock     = tic;
failsafeCount = 0;
serialOk      = true;
recFid        = -1;
recStart      = tic;
lastStep      = tic;
rxBuffer      = "";           % serial text not yet split into full lines
imuStatus     = "waiting";    % waiting / ok / calibrating / missing / lost
yawRate       = NaN;          % deg/s, + = turning right
yawRaw        = NaN;          % integrated yaw from the Arduino (deg)
headingZero   = 0;            % yawRaw value that counts as heading 0
accelG        = [NaN NaN NaN];
lastImuTime   = tic;

% ------------------------------------------------------------ session log
logFolder = fullfile(fileparts(mfilename("fullpath")), "logs");
if ~isfolder(logFolder)
    mkdir(logFolder);
end
logName = "drive_" + string(datetime("now", "Format", "yyyyMMdd_HHmmss")) + ".txt";
logFid = fopen(fullfile(logFolder, logName), "w");

% ---------------------------------------------------------------- connect
try
    s = serialport(port, 115200, "Timeout", 1);
catch err
    fclose(logFid);
    error("Could not open %s. Close the Arduino IDE / other MATLAB windows " + ...
          "and check the port with serialportlist.\n%s", port, err.message);
end
configureTerminator(s, "LF");
fprintf("Connected to %s. Waiting 3 s for the Uno to restart (keep the robot still)...\n", port);
pause(3);   % the Uno restarts and calibrates the gyro; its messages are read once the window opens

% --------------------------------------------------------------------- UI
fig = uifigure("Name", "Drive Control", "Position", [100 30 640 900]);
g = uigridlayout(fig, [12 2]);
g.RowHeight   = {50, 40, 55, 150, 25, 60, 32, 45, 40, 70, '1x', 40};
g.ColumnWidth = {'1x', '1x'};

banner = uilabel(g, "Text", "", "FontSize", 16, "FontWeight", "bold", ...
    "HorizontalAlignment", "center", "FontColor", "white");
banner.Layout.Row = 1; banner.Layout.Column = [1 2];

cmdLabel = uilabel(g, "Text", "", "FontSize", 20, "FontWeight", "bold", ...
    "HorizontalAlignment", "center");
cmdLabel.Layout.Row = 2; cmdLabel.Layout.Column = [1 2];

tickStep = 10 * (1 + (MAX_PCT > 50));
tickVals = -MAX_PCT:tickStep:MAX_PCT;
tickLabels = compose("%+d%%", tickVals);
tickLabels(tickVals == 0) = "STOP";
surgeSlider = uislider(g, "Limits", [-MAX_PCT MAX_PCT], "Value", 0, ...
    "MajorTicks", tickVals, "MajorTickLabels", cellstr(tickLabels), ...
    "ValueChangedFcn", @(src, ~) setSurge(src.Value));
surgeSlider.Layout.Row = 3; surgeSlider.Layout.Column = [1 2];

% Thruster panel laid out like the boat seen from above (front at top)
panel = uigridlayout(g, [3 2], "Padding", [0 0 0 0], "RowHeight", {20, '1x', '1x'});
panel.Layout.Row = 4; panel.Layout.Column = [1 2];
uilabel(panel, "Text", "LEFT hull   (front at top)", "HorizontalAlignment", "center", "FontWeight", "bold");
uilabel(panel, "Text", "RIGHT hull", "HorizontalAlignment", "center", "FontWeight", "bold");
thrLabels = gobjects(1, 4);
for idx = [1 3 2 4]   % grid fills row by row: LF, RF, LR, RR
    thrLabels(idx) = uilabel(panel, "Text", "", "FontSize", 16, "FontWeight", "bold", ...
        "HorizontalAlignment", "center", "FontColor", "white");
end

forceLabel = uilabel(g, "Text", "", "HorizontalAlignment", "center", "FontColor", [0.3 0.3 0.3]);
forceLabel.Layout.Row = 5; forceLabel.Layout.Column = [1 2];

% IMU (MPU-6050) readout + buttons
imuLabel = uilabel(g, "Text", "IMU: waiting for Arduino...", "FontSize", 15, ...
    "FontWeight", "bold", "FontName", "Consolas");
imuLabel.Layout.Row = 6; imuLabel.Layout.Column = 1;
sub = uigridlayout(g, [2 1], "Padding", [0 0 0 0], "RowSpacing", 4);
sub.Layout.Row = 6; sub.Layout.Column = 2;
uibutton(sub, "Text", "Calibrate gyro (keep robot still, 2 s)", ...
    "ButtonPushedFcn", @(~, ~) calibrateGyro());
uibutton(sub, "Text", "Zero heading", "ButtonPushedFcn", @(~, ~) zeroHeading());

numberField(7, 1, "Turn strength %", [0 TURN_MAX], turnPct, @(v) setTurnPct(v));
numberField(7, 2, "Straight trim % (+ = right)", [-TURN_MAX TURN_MAX], straightTrim, @(v) setTrim(v));

enableBtn = uibutton(g, "state", "Text", "Enable thrusters", "FontSize", 15, ...
    "ValueChangedFcn", @(src, ~) setEnabled(src.Value));
enableBtn.Layout.Row = 8; enableBtn.Layout.Column = 1;
sub = uigridlayout(g, [1 3], "Padding", [0 0 0 0], "ColumnWidth", {22, '1x', '1x'});
sub.Layout.Row = 8; sub.Layout.Column = 2;
pumpLamp = uilamp(sub, "Color", [0.5 0.5 0.5]);
pumpBtn = uibutton(sub, "state", "Text", "Impeller ON", ...
    "ValueChangedFcn", @(src, ~) setPump(src.Value));
uicheckbox(sub, "Text", "Record data", "ValueChangedFcn", @(src, ~) setRecording(src.Value));

btn = uibutton(g, "Text", "Turn step test (+turn, 0, -turn, 0)", ...
    "ButtonPushedFcn", @(~, ~) startStepTest());
btn.Layout.Row = 9; btn.Layout.Column = 1;
btn = uibutton(g, "Text", "Mark problem (m)", "ButtonPushedFcn", @(~, ~) markProblem());
btn.Layout.Row = 9; btn.Layout.Column = 2;

btn = uibutton(g, "Text", "STOP  (space bar)", "FontSize", 24, ...
    "FontWeight", "bold", "FontColor", "white", ...
    "BackgroundColor", [0.8 0.1 0.1], "ButtonPushedFcn", @(~, ~) stopAll());
btn.Layout.Row = 10; btn.Layout.Column = [1 2];

logArea = uitextarea(g, "Editable", "off", "FontName", "Consolas", "Value", "");
logArea.Layout.Row = 11; logArea.Layout.Column = [1 2];

hint = uilabel(g, "WordWrap", "on", "HorizontalAlignment", "center", "FontColor", [0.4 0.4 0.4], ...
    "Text", "Click an empty area first.  Up/Down = speed   hold Left/Right = turn   " + ...
            "0 = ramp to stop   Space = STOP   i = impeller   m = mark");
hint.Layout.Row = 12; hint.Layout.Column = [1 2];

fig.WindowKeyPressFcn   = @onKeyPress;
fig.WindowKeyReleaseFcn = @onKeyRelease;
fig.CloseRequestFcn     = @(~, ~) onClose();

surgeSlider.Enable = false;
updateDisplay();
setBanner("1) Main switch ON (ESCs beep)   2) Enable thrusters", [0.3 0.3 0.3]);
logEvent("Connected to " + port + ". Log file: logs\" + logName);

sendTimer = timer("ExecutionMode", "fixedSpacing", "Period", SEND_PERIOD, ...
    "BusyMode", "drop", "TimerFcn", @(~, ~) stepAndSend());
start(sendTimer);

% ================================================================ helpers
    function numberField(row, col, text, limits, value, onChange)
        box = uigridlayout(g, [1 2], "Padding", [0 0 0 0], "ColumnWidth", {'fit', '1x'});
        box.Layout.Row = row; box.Layout.Column = col;
        uilabel(box, "Text", text);
        uieditfield(box, "numeric", "Limits", limits, "Value", value, ...
            "ValueChangedFcn", @(src, ~) onChange(src.Value));
    end

    % ---------------------------------------------------------------- mixer
    function T = thrustOf(u)
        % Throttle fraction (-1..1) -> thrust as a fraction of max forward thrust
        if u >= 0
            T = u^2;
        else
            T = -REV_RATIO * u^2;
        end
    end

    function u = throttleOf(T)
        % Inverse of thrustOf
        if T >= 0
            u = sqrt(T);
        else
            u = -sqrt(-T / REV_RATIO);
        end
    end

    function turn = turnCommand()
        turn = turnKey * turnPct + straightTrim;
    end

    function mix()
        % surge + turn (in thrust) -> left/right throttle -> 4 thruster targets
        if ~enabled
            targetPct = [0 0 0 0];
            return
        end
        turn = turnCommand();
        surgeT = thrustOf(surgePct / 100);
        turnT = sign(turn) * (turn / 100)^2;
        leftPct  = limitPct(100 * throttleOf(surgeT + turnT));
        rightPct = limitPct(100 * throttleOf(surgeT - turnT));
        targetPct = [leftPct leftPct rightPct rightPct];
    end

    function v = limitPct(v)
        v = snapSpeed(max(-MAX_PCT, min(MAX_PCT, v)));
    end

    function v = snapSpeed(v)
        % Skip the unstable zone: near 0 -> 0, otherwise at least MIN_PCT
        if abs(v) < MIN_PCT / 2
            v = 0;
        elseif abs(v) < MIN_PCT
            v = sign(v) * MIN_PCT;
        end
    end

    % ------------------------------------------------------ send / ramp
    function stepAndSend()
        dt = toc(lastStep);
        lastStep = tic;
        updateStepTest();
        mix();
        if enabled
            for k = 1:4
                actualPct(k) = rampToward(actualPct(k), targetPct(k), dt);
            end
        else
            actualPct = [0 0 0 0];
        end
        pulse = round(arrayfun(@pctToPulse, 1:4, actualPct));
        sendLine();
        updateDisplay();
    end

    function a = rampToward(a, t, dt)
        % Ramp a toward t; reversing first ramps down to 0, then up the other way.
        if a ~= 0 && t ~= 0 && sign(t) ~= sign(a)
            goal = 0;
        else
            goal = t;
        end
        if abs(goal) > abs(a)
            rate = RAMP_UP;
        else
            rate = RAMP_DOWN;
        end
        if rate <= 0
            a = goal;
        else
            stepMax = rate * dt;
            a = a + max(-stepMax, min(stepMax, goal - a));
        end
        if a ~= 0 && abs(a) < MIN_PCT
            if goal == 0
                a = 0;
            else
                a = sign(goal) * MIN_PCT;
            end
        end
    end

    function p = pctToPulse(i, pct)
        if REVERSED(i)
            pct = -pct;
        end
        if pct > 0
            p = NEUTRAL + TRIM_POS(i) + pct * US_PER_PCT;
        elseif pct < 0
            p = NEUTRAL - TRIM_NEG(i) + pct * US_PER_PCT;
        else
            p = NEUTRAL;
        end
        p = max(NEUTRAL - MAX_OFFSET, min(NEUTRAL + MAX_OFFSET, p));
    end

    function sendLine()
        try
            writeline(s, sprintf("%d,%d,%d,%d,%d", [pulse pump]));
            if recFid > 0
                [roll, pitch] = tiltDeg();
                fprintf(recFid, "%.3f,%g,%g,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%d,%d," + ...
                    "%.2f,%.1f,%.1f,%.1f,%.3f,%.3f,%.3f\n", ...
                    toc(recStart), surgePct, turnCommand(), actualPct, pulse, pump, ...
                    yawRate, headingDeg(), roll, pitch, accelG);
            end
            if ~serialOk
                serialOk = true;
                logEvent("USB link restored.");
                refreshBanner();
            end
            n = s.NumBytesAvailable;
            if n > 0
                % Keep any incomplete last line for the next read
                rxBuffer = rxBuffer + string(read(s, n, "char"));
                parts = split(rxBuffer, newline);
                rxBuffer = parts(end);
                handleReplies(strtrim(parts(1:end-1)));
            end
            if imuStatus == "ok" && toc(lastImuTime) > 1
                imuStatus = "lost";
                logEvent("IMU data stopped arriving.");
            end
        catch err
            if serialOk
                serialOk = false;
                logEvent("USB/serial ERROR: " + err.message);
                setBanner("USB CONNECTION LOST - check the cable", [0.8 0.1 0.1]);
            end
        end
    end

    function handleReplies(replies)
        for r = replies'
            if startsWith(r, "IMU,")
                parseImu(r);
                continue
            end
            switch r
                case "IMU OK"
                    logEvent("IMU found (MPU-6050).");
                case "BRIDGE READY"
                    logEvent("Shore radio bridge ready - waiting for the boat.");
                case "RADIO HW MISSING"
                    logEvent("Shore bridge: NRF24 not responding (check its wiring / power).");
                case "RELAY READY"
                    logEvent("Boat Pi relay connected.");
                case "RADIO OK"
                    logEvent("RADIO OK: the boat is acknowledging commands.");
                    refreshBanner();
                case "RADIO LOST"
                    logEvent("RADIO LOST: no acknowledgement from the boat (out of range / boat off).");
                    setBanner("RADIO LOST - boat stops by itself within 0.3 s", [0.8 0.1 0.1]);
                case "IMU MISSING"
                    imuStatus = "missing";
                    logEvent("IMU MISSING: check MPU-6050 wiring (SDA A4, SCL A5, 5V, GND).");
                case "IMU LOST"
                    imuStatus = "lost";
                    logEvent("IMU LOST: the Arduino stopped getting readings (loose wire?).");
                case "CAL START"
                    imuStatus = "calibrating";
                    logEvent("Gyro calibration started - keep the robot still for 2 s.");
                case "CAL DONE"
                    imuStatus = "ok";
                    headingZero = 0;
                    lastImuTime = tic;
                    logEvent("Gyro calibration done. Heading zeroed.");
                case "PUMP ON"
                    pumpLamp.Color = [0.1 0.8 0.1];
                    logEvent("Impeller ON (confirmed by Arduino).");
                case "PUMP OFF"
                    pumpLamp.Color = [0.5 0.5 0.5];
                    logEvent("Impeller OFF (confirmed by Arduino).");
                case "FAILSAFE"
                    failsafeCount = failsafeCount + 1;
                    logEvent(sprintf("Arduino FAILSAFE #%d: commands interrupted > 0.5 s.", ...
                        failsafeCount));
                    refreshBanner();
            end
        end
    end

    % ------------------------------------------------------------------ IMU
    function parseImu(txt)
        % "IMU,<yaw rate>,<yaw>,<ax>,<ay>,<az>"
        v = str2double(split(extractAfter(txt, "IMU,"), ","));
        if numel(v) ~= 5 || any(isnan(v))
            return
        end
        yawRate = YAW_SIGN * v(1);
        yawRaw  = YAW_SIGN * v(2);
        accelG  = v(3:5)';
        lastImuTime = tic;
        if imuStatus ~= "ok"
            imuStatus = "ok";
        end
    end

    function h = headingDeg()
        % Heading relative to the last zero, wrapped to -180..180 (+ = right)
        h = mod(yawRaw - headingZero + 180, 360) - 180;
    end

    function [roll, pitch] = tiltDeg()
        % Tilt from gravity (assumes the MPU is mounted flat)
        roll  = atan2d(accelG(2), accelG(3));
        pitch = atan2d(-accelG(1), hypot(accelG(2), accelG(3)));
    end

    function calibrateGyro()
        if any(abs(actualPct) > 0)
            logEvent("Stop the thrusters before calibrating the gyro.");
            return
        end
        try
            writeline(s, "CAL");
        catch err
            logEvent("Could not send CAL: " + err.message);
        end
    end

    function zeroHeading()
        if ~isnan(yawRaw)
            headingZero = yawRaw;
            logEvent("Heading zeroed.");
        end
    end

    function updateImuDisplay()
        switch imuStatus
            case "ok"
                [roll, pitch] = tiltDeg();
                imuLabel.Text = sprintf("Yaw rate %+6.1f °/s\nHeading  %+6.1f °\nRoll %+5.1f°  Pitch %+5.1f°", ...
                    yawRate, headingDeg(), roll, pitch);
                imuLabel.FontColor = [0 0 0];
            case "calibrating"
                imuLabel.Text = "IMU: calibrating..." + newline + "keep the robot still";
                imuLabel.FontColor = [0.2 0.4 0.8];
            case "missing"
                imuLabel.Text = "IMU MISSING" + newline + "check SDA A4, SCL A5, 5V, GND";
                imuLabel.FontColor = [0.8 0.1 0.1];
            case "lost"
                imuLabel.Text = "IMU LOST" + newline + "check the wiring";
                imuLabel.FontColor = [0.8 0.1 0.1];
            otherwise
                imuLabel.Text = "IMU: waiting for Arduino..." + newline + ...
                    "(re-upload four_thrusters.ino if this stays)";
                imuLabel.FontColor = [0.4 0.4 0.4];
        end
    end

    % ------------------------------------------------------------ commands
    function setSurge(v)
        if ~enabled
            surgeSlider.Value = surgePct;
            return
        end
        surgePct = snapSpeed(max(-MAX_PCT, min(MAX_PCT, round(v))));
        surgeSlider.Value = surgePct;
        stepAndSend();   % send now instead of waiting for the timer
    end

    function stepSurge(d)
        % From 0 jump straight to +/-MIN_PCT; stepping back inside the
        % unstable zone (or through zero) goes to 0.
        if surgePct == 0
            next = sign(d) * MIN_PCT;
        elseif abs(surgePct + d) < MIN_PCT || sign(surgePct + d) ~= sign(surgePct)
            next = 0;
        else
            next = surgePct + d;
        end
        setSurge(next);
    end

    function setTurn(dir)
        if stepIdx > 0 || turnKey == dir
            return
        end
        turnKey = dir;
        stepAndSend();
    end

    function setTurnPct(v)
        turnPct = v;
        logEvent(sprintf("Turn strength set to %g %%.", v));
    end

    function setTrim(v)
        straightTrim = v;
        logEvent(sprintf("Straight trim set to %+g %%.", v));
    end

    function setEnabled(on)
        stopStepTest();
        zeroAll();
        enabled = on;
        surgeSlider.Enable = on;
        if on
            enableBtn.Text = "Disable thrusters";
            logEvent("Thrusters ENABLED.");
        else
            enableBtn.Text = "Enable thrusters";
            logEvent("Thrusters disabled.");
        end
        refreshBanner();
        stepAndSend();
    end

    function zeroAll()
        % Instant (not ramped) - used by STOP and enable/disable
        surgePct = 0;
        turnKey = 0;
        surgeSlider.Value = 0;
        targetPct = [0 0 0 0];
        actualPct = [0 0 0 0];
        pulse = NEUTRAL * ones(1, 4);
    end

    function setPump(on)
        pump = double(on);
        if on
            pumpBtn.Text = "Impeller OFF";
        else
            pumpBtn.Text = "Impeller ON";
        end
        stepAndSend();
    end

    function stopAll()
        stopStepTest();
        zeroAll();
        enabled = false;
        enableBtn.Value = false;
        enableBtn.Text = "Enable thrusters";
        surgeSlider.Enable = false;
        pump = 0;
        pumpBtn.Value = false;
        pumpBtn.Text = "Impeller ON";
        stepAndSend();
        logEvent("STOP pressed: thrusters at 1500, impeller off.");
        setBanner("STOPPED  -  press Enable thrusters to continue", [0.8 0.1 0.1]);
    end

    function markProblem()
        logEvent(sprintf("*** PROBLEM MARKED ***  surge %+d %%, turn %+g %%, " + ...
            "thrusters %+.0f/%+.0f/%+.0f/%+.0f %%, yaw rate %+.1f deg/s, heading %+.0f deg", ...
            surgePct, turnCommand(), actualPct, yawRate, headingDeg()));
    end

    % ---------------------------------------------------- turn step test
    function startStepTest()
        if ~enabled
            logEvent("Turn step test needs thrusters enabled first.");
            return
        end
        stepIdx = 1;
        stepClock = tic;
        turnKey = stepSeq(1);
        logEvent(sprintf("TURN STEP TEST started at surge %+d %%, turn %g %%, %g s per phase.", ...
            surgePct, turnPct, STEP_TIME));
        announceStep();
    end

    function updateStepTest()
        if stepIdx == 0 || toc(stepClock) < STEP_TIME
            return
        end
        stepIdx = stepIdx + 1;
        stepClock = tic;
        if stepIdx > numel(stepSeq)
            stopStepTest();
            logEvent("TURN STEP TEST finished.");
            refreshBanner();
            return
        end
        turnKey = stepSeq(stepIdx);
        announceStep();
    end

    function announceStep()
        names = ["turn LEFT", "straight", "turn RIGHT"];
        msg = sprintf("STEP TEST %d/%d: %s", stepIdx, numel(stepSeq), names(turnKey + 2));
        setBanner(msg, [0.2 0.4 0.8]);
        logEvent(msg);
    end

    function stopStepTest()
        if stepIdx > 0
            stepIdx = 0;
            turnKey = 0;
        end
    end

    % ------------------------------------------------------------ recording
    function setRecording(on)
        if on
            name = "drivedata_" + string(datetime("now", "Format", "yyyyMMdd_HHmmss")) + ".csv";
            recFid = fopen(fullfile(logFolder, name), "w");
            fprintf(recFid, "t_s,surge_pct,turn_pct,LF_pct,LR_pct,RF_pct,RR_pct," + ...
                "LF_us,LR_us,RF_us,RR_us,impeller," + ...
                "yaw_rate_dps,heading_deg,roll_deg,pitch_deg,ax_g,ay_g,az_g\n");
            recStart = tic;
            logEvent("Recording data to logs\" + name);
        elseif recFid > 0
            fclose(recFid);
            recFid = -1;
            logEvent("Recording stopped.");
        end
    end

    % ----------------------------------------------------------------- UI
    function updateDisplay()
        turn = turnCommand();
        if turn < 0
            turnText = sprintf("<< LEFT %g %%", -turn);
        elseif turn > 0
            turnText = sprintf("RIGHT %g %% >>", turn);
        else
            turnText = "straight";
        end
        cmdLabel.Text = sprintf("Speed %+d %%     Turn: %s", surgePct, turnText);
        updateImuDisplay();

        for k = 1:4
            v = actualPct(k);
            thrLabels(k).Text = sprintf("%s (%s)\n%+.0f %%   %d us", NAMES(k), PINS(k), v, pulse(k));
            if v > 0
                thrLabels(k).BackgroundColor = [0.1 0.35 + 0.5 * v / MAX_PCT 0.2];
            elseif v < 0
                thrLabels(k).BackgroundColor = [0.6 + 0.35 * -v / MAX_PCT 0.35 0.05];
            else
                thrLabels(k).BackgroundColor = [0.45 0.45 0.45];
            end
        end

        % Estimated forces from the thrust model (both thrusters on each side)
        leftT  = thrustOf(actualPct(1) / 100) + thrustOf(actualPct(2) / 100);
        rightT = thrustOf(actualPct(3) / 100) + thrustOf(actualPct(4) / 100);
        if isnan(MAX_THRUST_N)
            forceLabel.Text = sprintf("Model: forward thrust %.2f, turning moment %.3f " + ...
                "(x one thruster's max thrust; set MAX_THRUST_N for N and N·m)", ...
                leftT + rightT, (leftT - rightT) * HULL_SPACING / 2);
        else
            forceLabel.Text = sprintf("Model: forward thrust %.1f N, turning moment %.2f N·m", ...
                (leftT + rightT) * MAX_THRUST_N, (leftT - rightT) * HULL_SPACING / 2 * MAX_THRUST_N);
        end
    end

    function refreshBanner()
        if ~serialOk
            return
        end
        if enabled
            txt = "ENABLED - Up/Down = speed, hold Left/Right = turn";
            color = [0.1 0.6 0.2];
        else
            txt = "Thrusters disabled  -  press Enable thrusters";
            color = [0.3 0.3 0.3];
        end
        if failsafeCount > 0
            txt = txt + sprintf("   (link dropouts: %d)", failsafeCount);
        end
        setBanner(txt, color);
    end

    function setBanner(txt, color)
        banner.Text = txt;
        banner.BackgroundColor = color;
    end

    function logEvent(msg)
        entry = string(datetime("now", "Format", "HH:mm:ss")) + "  " + msg;
        old = string(logArea.Value);
        old = old(:);
        old(old == "") = [];
        logArea.Value = cellstr([entry; old(1:min(end, 199))]);   % newest on top
        if logFid > 0
            fprintf(logFid, "%s\n", entry);
        end
    end

    function onKeyPress(~, evt)
        switch evt.Key
            case "space"
                stopAll();
            case "uparrow"
                stepSurge(SURGE_STEP);
            case "downarrow"
                stepSurge(-SURGE_STEP);
            case "leftarrow"
                setTurn(-1);
            case "rightarrow"
                setTurn(1);
            case {"0", "numpad0"}
                setSurge(0);
            case "i"
                pumpBtn.Value = ~pumpBtn.Value;
                setPump(pumpBtn.Value);
            case "m"
                markProblem();
        end
    end

    function onKeyRelease(~, evt)
        % Releasing Left/Right springs back to straight, like a joystick
        if (evt.Key == "leftarrow" && turnKey == -1) || (evt.Key == "rightarrow" && turnKey == 1)
            setTurn(0);
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
        logEvent("Window closed. Thrusters stopped, impeller off.");
        if recFid > 0
            fclose(recFid);
        end
        fclose(logFid);
        delete(sendTimer);
        delete(s);
        delete(fig);
        fprintf("Stopped. Session log saved to %s\n", fullfile(logFolder, logName));
    end
end
