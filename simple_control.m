function simple_control(port)
%SIMPLE_CONTROL  Simple GUI: one speed for all 4 thrusters + impeller.
%   simple_control("COM5")
%
%   Talks to four_thrusters.ino on the Arduino Uno (same as
%   four_thruster_control.m, which keeps the per-thruster / turning controls).
%
%   Speed changes are ramped: the slider sets the TARGET speed, and the
%   ACTUAL speed sent to the thrusters moves toward it at the ramp-up rate
%   (speeding up) or ramp-down rate (slowing down), in % per second.
%   STOP is always instant; "0 %" ramps down smoothly.
%
%   "Run ramp profile" ramps up to a peak, holds, and ramps back to 0.
%   "Record data" saves target/actual/pulses at 20 Hz to logs\data_*.csv.
%
%   Keys:  Up/Down = speed +/-1 %   Space = STOP   i = impeller   m = mark problem
%
%   For connection tests: "Test each thruster" runs one thruster at a time so
%   you can wiggle its wires, and "Mark problem" writes a timestamped note.
%   Every event is also saved to logs\session_<date>_<time>.txt.

if nargin < 1
    port = "COM5";
end

NEUTRAL      = 1500;  % us, stop
MAX_PCT      = 100;   % speed slider range in % throttle
US_PER_PCT   = 5;     % 500 us from neutral = 100 %
MAX_OFFSET   = 500;   % hard limit on every pulse: 1500 +/- 500 us
SEND_PERIOD  = 0.05;  % s between commands (20 Hz, inside the 0.5 s failsafe)
RAMP_UP      = 20;    % %/s when speeding up   (0 = instant)
RAMP_DOWN    = 40;    % %/s when slowing down  (0 = instant)
STEP_PCT     = 5;     % speed change for the -5 % / +5 % buttons
MIN_PCT      = 10;    % slowest non-zero speed; below this the motors stutter
TEST_PCT     = 10;    % speed used by "Test each thruster"
TEST_TIME    = 3;     % s each thruster runs during the test
PROFILE_PEAK = 30;    % default peak % for "Run ramp profile"
PROFILE_HOLD = 3;     % default hold time (s) at the peak

% Per thruster, order: left front, left rear, right front, right rear
NAMES    = ["Left front", "Left rear", "Right front", "Right rear"];
PINS     = ["D9", "D10", "D5", "D11"];
REVERSED = [false false false false]; % true = flip this thruster's direction
TRIM_POS = [0 0 0 0];                 % start trim above 1500 (us)
TRIM_NEG = [0 0 0 0];                 % start trim below 1500 (us)

speedPct      = 0;                    % target speed shown on the slider
targetPct     = [0 0 0 0];            % per-thruster target (future controllers set this)
actualPct     = [0 0 0 0];            % per-thruster ramped speed actually sent
pulse         = NEUTRAL * ones(1, 4);
pump          = 0;
enabled       = false;
testing       = false;
testIdx       = 0;
rampUp        = RAMP_UP;
rampDown      = RAMP_DOWN;
profilePhase  = "";                   % "", "up", "hold" or "down"
profilePeak   = PROFILE_PEAK;
profileHold   = PROFILE_HOLD;
profileClock  = tic;
failsafeCount = 0;
serialOk      = true;
recFid        = -1;
recStart      = tic;
lastStep      = tic;

% ------------------------------------------------------------ session log
logFolder = fullfile(fileparts(mfilename("fullpath")), "logs");
if ~isfolder(logFolder)
    mkdir(logFolder);
end
logName = "session_" + string(datetime("now", "Format", "yyyyMMdd_HHmmss")) + ".txt";
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
fprintf("Connected to %s. Waiting 3 s for the Uno to restart...\n", port);
pause(3);
flush(s);

% --------------------------------------------------------------------- UI
fig = uifigure("Name", "Robot Control", "Position", [100 40 600 790]);
g = uigridlayout(fig, [11 3]);
g.RowHeight   = {50, 45, 55, 40, 32, 32, 45, 40, 70, '1x', 22};
g.ColumnWidth = {'1x', '1x', '1x'};

banner = uilabel(g, "Text", "", "FontSize", 16, "FontWeight", "bold", ...
    "HorizontalAlignment", "center", "FontColor", "white");
banner.Layout.Row = 1; banner.Layout.Column = [1 3];

speedLabel = uilabel(g, "Text", "", "FontSize", 22, "FontWeight", "bold", ...
    "HorizontalAlignment", "center");
speedLabel.Layout.Row = 2; speedLabel.Layout.Column = [1 3];

% 0 = STOP baseline; anything between 0 and +/-MIN_PCT snaps to 0 or +/-MIN_PCT
tickStep = 10 * (1 + (MAX_PCT > 50));
tickVals = -MAX_PCT:tickStep:MAX_PCT;
tickLabels = compose("%+d%%", tickVals);
tickLabels(tickVals == 0) = "STOP";
speedSlider = uislider(g, "Limits", [-MAX_PCT MAX_PCT], "Value", 0, ...
    "MajorTicks", tickVals, "MajorTickLabels", cellstr(tickLabels), ...
    "MinorTicks", -MAX_PCT:5:MAX_PCT, ...
    "ValueChangingFcn", @(~, evt) setSpeed(evt.Value, true), ...
    "ValueChangedFcn",  @(src, ~) setSpeed(src.Value, false));
speedSlider.Layout.Row = 3; speedSlider.Layout.Column = [1 3];

btn = uibutton(g, "Text", sprintf("- %d %%", STEP_PCT), "FontSize", 16, ...
    "ButtonPushedFcn", @(~, ~) stepSpeed(-STEP_PCT));
btn.Layout.Row = 4; btn.Layout.Column = 1;
btn = uibutton(g, "Text", "0 %  (ramp down)", "FontSize", 16, ...
    "ButtonPushedFcn", @(~, ~) setSpeed(0, false));
btn.Layout.Row = 4; btn.Layout.Column = 2;
btn = uibutton(g, "Text", sprintf("+ %d %%", STEP_PCT), "FontSize", 16, ...
    "ButtonPushedFcn", @(~, ~) stepSpeed(STEP_PCT));
btn.Layout.Row = 4; btn.Layout.Column = 3;

% Ramp rates + data recording
numberField(5, 1, "Ramp up %/s", [0 1000], rampUp, @(v) setRampUp(v));
numberField(5, 2, "Ramp down %/s", [0 1000], rampDown, @(v) setRampDown(v));
recBox = uicheckbox(g, "Text", "Record data (CSV)", ...
    "ValueChangedFcn", @(src, ~) setRecording(src.Value));
recBox.Layout.Row = 5; recBox.Layout.Column = 3;

% Ramp profile
numberField(6, 1, "Profile peak %", [-MAX_PCT MAX_PCT], profilePeak, @(v) setProfilePeak(v));
numberField(6, 2, "Hold s", [0 600], profileHold, @(v) setProfileHold(v));
btn = uibutton(g, "Text", "Run ramp profile", "FontWeight", "bold", ...
    "ButtonPushedFcn", @(~, ~) startProfile());
btn.Layout.Row = 6; btn.Layout.Column = 3;

enableBtn = uibutton(g, "state", "Text", "Enable thrusters", "FontSize", 15, ...
    "ValueChangedFcn", @(src, ~) setEnabled(src.Value));
enableBtn.Layout.Row = 7; enableBtn.Layout.Column = [1 2];
sub = uigridlayout(g, [1 2], "Padding", [0 0 0 0], "ColumnWidth", {22, '1x'});
sub.Layout.Row = 7; sub.Layout.Column = 3;
pumpLamp = uilamp(sub, "Color", [0.5 0.5 0.5]);
pumpBtn = uibutton(sub, "state", "Text", "Impeller ON", "FontSize", 15, ...
    "ValueChangedFcn", @(src, ~) setPump(src.Value));

btn = uibutton(g, "Text", "Test each thruster", ...
    "ButtonPushedFcn", @(~, ~) startTest());
btn.Layout.Row = 8; btn.Layout.Column = [1 2];
btn = uibutton(g, "Text", "Mark problem (m)", ...
    "ButtonPushedFcn", @(~, ~) markProblem());
btn.Layout.Row = 8; btn.Layout.Column = 3;

btn = uibutton(g, "Text", "STOP  (space bar)", "FontSize", 24, ...
    "FontWeight", "bold", "FontColor", "white", ...
    "BackgroundColor", [0.8 0.1 0.1], "ButtonPushedFcn", @(~, ~) stopAll());
btn.Layout.Row = 9; btn.Layout.Column = [1 3];

logArea = uitextarea(g, "Editable", "off", "FontName", "Consolas", "Value", "");
logArea.Layout.Row = 10; logArea.Layout.Column = [1 3];

hint = uilabel(g, "Text", "Keys:  Up/Down = speed ±1 %    Space = STOP    i = impeller    m = mark problem", ...
    "HorizontalAlignment", "center", "FontColor", [0.4 0.4 0.4]);
hint.Layout.Row = 11; hint.Layout.Column = [1 3];

fig.WindowKeyPressFcn = @onKey;
fig.CloseRequestFcn   = @(~, ~) onClose();

speedSlider.Enable = false;
updateDisplay();
setBanner("1) Main switch ON (ESCs beep)   2) Enable thrusters", [0.3 0.3 0.3]);
logEvent("Connected to " + port + ". Log file: logs\" + logName);

% ----------------------------------------------------------------- timers
testTimer = timer("ExecutionMode", "fixedSpacing", "Period", TEST_TIME, ...
    "TimerFcn", @(~, ~) testTick());
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

    function stepAndSend()
        dt = toc(lastStep);
        lastStep = tic;
        if enabled
            updateProfile();
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
        % Move actual speed a toward target t using the ramp-up/down rates.
        % Reversing direction first ramps down to 0, then up the other way.
        if a ~= 0 && t ~= 0 && sign(t) ~= sign(a)
            goal = 0;
        else
            goal = t;
        end
        if abs(goal) > abs(a)
            rate = rampUp;
        else
            rate = rampDown;
        end
        if rate <= 0
            a = goal;
        else
            stepMax = rate * dt;
            a = a + max(-stepMax, min(stepMax, goal - a));
        end
        % Skip the unstable zone below MIN_PCT
        if a ~= 0 && abs(a) < MIN_PCT
            if goal == 0
                a = 0;
            else
                a = sign(goal) * MIN_PCT;
            end
        end
    end

    function sendLine()
        try
            writeline(s, sprintf("%d,%d,%d,%d,%d", [pulse pump]));
            if recFid > 0
                fprintf(recFid, "%.3f,%g,%.2f,%d,%d,%d,%d,%d\n", toc(recStart), ...
                    speedPct, mean(actualPct), pulse, pump);
            end
            if ~serialOk
                serialOk = true;
                logEvent("USB link restored.");
                refreshBanner();
            end
            n = s.NumBytesAvailable;
            if n > 0
                handleReplies(strtrim(splitlines(string(read(s, n, "char")))));
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
            switch r
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

    % ------------------------------------------------------------ commands
    function setSpeed(v, fromSlider)
        if profilePhase ~= "" && ~fromSlider && v == 0 && enabled
            stopProfile();   % "0 %" button cancels a profile and ramps down
            logEvent("PROFILE cancelled: ramping down to 0.");
            refreshBanner();
        end
        if ~enabled || testing || profilePhase ~= ""
            speedSlider.Value = speedPct;
            return
        end
        speedPct = snapSpeed(max(-MAX_PCT, min(MAX_PCT, round(v))));
        targetPct = speedPct * ones(1, 4);
        if ~fromSlider
            speedSlider.Value = speedPct;
        end
        stepAndSend();   % send now instead of waiting for the timer
    end

    function v = snapSpeed(v)
        % Skip the unstable zone: near 0 -> STOP, otherwise at least MIN_PCT
        if abs(v) < MIN_PCT / 2
            v = 0;
        elseif abs(v) < MIN_PCT
            v = sign(v) * MIN_PCT;
        end
    end

    function stepSpeed(d)
        % Buttons / arrow keys: from STOP jump straight to +/-MIN_PCT,
        % and stepping back inside the unstable zone goes to STOP.
        if speedPct == 0
            next = sign(d) * MIN_PCT;
        elseif abs(speedPct + d) < MIN_PCT || sign(speedPct + d) ~= sign(speedPct)
            next = 0;
        else
            next = speedPct + d;
        end
        setSpeed(next, false);
    end

    function zeroSpeed()
        % Instant (not ramped) - used by STOP, enable/disable and tests
        speedPct = 0;
        targetPct = [0 0 0 0];
        actualPct = [0 0 0 0];
        speedSlider.Value = 0;
        pulse = NEUTRAL * ones(1, 4);
        updateDisplay();
    end

    function setEnabled(on)
        stopTest();
        stopProfile();
        zeroSpeed();
        enabled = on;
        speedSlider.Enable = on;
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
        stopTest();
        stopProfile();
        zeroSpeed();
        enabled = false;
        enableBtn.Value = false;
        enableBtn.Text = "Enable thrusters";
        speedSlider.Enable = false;
        pump = 0;
        pumpBtn.Value = false;
        pumpBtn.Text = "Impeller ON";
        stepAndSend();
        logEvent("STOP pressed: thrusters at 1500, impeller off.");
        setBanner("STOPPED  -  press Enable thrusters to continue", [0.8 0.1 0.1]);
    end

    function markProblem()
        logEvent(sprintf("*** PROBLEM MARKED ***  target %+d %%, actual %+.0f %%, " + ...
            "pulses %d/%d/%d/%d, impeller %d", speedPct, mean(actualPct), pulse, pump));
    end

    % ------------------------------------------------------ ramp settings
    function setRampUp(v)
        rampUp = v;
        logEvent(sprintf("Ramp up set to %g %%/s.", v));
    end

    function setRampDown(v)
        rampDown = v;
        logEvent(sprintf("Ramp down set to %g %%/s.", v));
    end

    function setProfilePeak(v)
        profilePeak = snapSpeed(round(v));
    end

    function setProfileHold(v)
        profileHold = v;
    end

    % ------------------------------------------------------- ramp profile
    function startProfile()
        if ~enabled || testing
            logEvent("Ramp profile needs thrusters enabled (and no test running).");
            return
        end
        if profilePeak == 0
            logEvent("Set a non-zero profile peak first.");
            return
        end
        speedPct = profilePeak;
        speedSlider.Value = speedPct;
        targetPct = profilePeak * ones(1, 4);
        profilePhase = "up";
        speedSlider.Enable = false;
        msg = sprintf("PROFILE: ramping up to %+d %% at %g %%/s", profilePeak, rampUp);
        setBanner(msg, [0.2 0.4 0.8]);
        logEvent(msg);
    end

    function updateProfile()
        % Called every tick while enabled; advances up -> hold -> down -> done
        switch profilePhase
            case "up"
                if all(abs(actualPct - targetPct) < 0.5)
                    profilePhase = "hold";
                    profileClock = tic;
                    msg = sprintf("PROFILE: holding %+d %% for %g s", profilePeak, profileHold);
                    setBanner(msg, [0.2 0.4 0.8]);
                    logEvent(msg);
                end
            case "hold"
                if toc(profileClock) >= profileHold
                    profilePhase = "down";
                    speedPct = 0;
                    speedSlider.Value = 0;
                    targetPct = [0 0 0 0];
                    msg = sprintf("PROFILE: ramping down to 0 at %g %%/s", rampDown);
                    setBanner(msg, [0.2 0.4 0.8]);
                    logEvent(msg);
                end
            case "down"
                if all(actualPct == 0)
                    stopProfile();
                    logEvent("PROFILE finished.");
                    refreshBanner();
                end
        end
    end

    function stopProfile()
        if profilePhase ~= ""
            profilePhase = "";
            speedSlider.Enable = enabled && ~testing;
        end
    end

    % ------------------------------------------------------------ recording
    function setRecording(on)
        if on
            name = "data_" + string(datetime("now", "Format", "yyyyMMdd_HHmmss")) + ".csv";
            recFid = fopen(fullfile(logFolder, name), "w");
            fprintf(recFid, "t_s,target_pct,actual_pct,LF_us,LR_us,RF_us,RR_us,impeller\n");
            recStart = tic;
            logEvent("Recording data to logs\" + name);
        elseif recFid > 0
            fclose(recFid);
            recFid = -1;
            logEvent("Recording stopped.");
        end
    end

    % --------------------------------------------------- thruster test
    function startTest()
        if ~enabled
            logEvent("Test needs thrusters enabled first.");
            return
        end
        stopProfile();
        zeroSpeed();
        testing = true;
        testIdx = 0;
        speedSlider.Enable = false;
        logEvent(sprintf("Thruster test started: each runs %d s at +%d %%.", TEST_TIME, TEST_PCT));
        stop(testTimer);
        start(testTimer);   % first tick runs immediately
    end

    function testTick()
        testIdx = testIdx + 1;
        if testIdx > 4
            stopTest();
            logEvent("Thruster test finished.");
            refreshBanner();
            return
        end
        targetPct = [0 0 0 0];
        targetPct(testIdx) = TEST_PCT;
        msg = sprintf("TEST: %s (%s) should spin now", NAMES(testIdx), PINS(testIdx));
        setBanner(msg, [0.85 0.5 0.0]);
        logEvent(msg);
    end

    function stopTest()
        if testing
            stop(testTimer);
            testing = false;
            targetPct = [0 0 0 0];
            speedSlider.Enable = enabled;
        end
    end

    % ----------------------------------------------------------------- UI
    function updateDisplay()
        actual = mean(actualPct);
        if testing
            speedLabel.Text = "Test running";
        elseif speedPct == 0 && all(actualPct == 0)
            speedLabel.Text = "STOP  (0 % baseline)";
        elseif abs(actual - speedPct) < 0.5
            speedLabel.Text = sprintf("Speed (all 4):  %+d %%", speedPct);
        elseif abs(speedPct) > abs(actual)
            speedLabel.Text = sprintf("Ramping up:  %+.0f %%  ->  %+d %%", actual, speedPct);
        else
            speedLabel.Text = sprintf("Ramping down:  %+.0f %%  ->  %+d %%", actual, speedPct);
        end
    end

    function refreshBanner()
        if ~serialOk
            return
        end
        if enabled
            txt = "ENABLED - use the slider, buttons or Up/Down keys";
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

    function onKey(~, evt)
        switch evt.Key
            case "space"
                stopAll();
            case "uparrow"
                stepSpeed(1);
            case "downarrow"
                stepSpeed(-1);
            case "i"
                pumpBtn.Value = ~pumpBtn.Value;
                setPump(pumpBtn.Value);
            case "m"
                markProblem();
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
        delete([sendTimer, testTimer]);
        delete(s);
        delete(fig);
        fprintf("Stopped. Session log saved to %s\n", fullfile(logFolder, logName));
    end
end
