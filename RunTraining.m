%% === BAGIAN 1: MAIN SCRIPT ===
clc; clear; close all;

% === CONFIG ===
USE_PARALLEL = true;  % Parallel Mode ON
REAL_TIME_LIMIT = 90; % Timeout per simulation

% 1. BERSIH-BERSIH
disp('🧹 Cleaning Memory...');
delete(gcp('nocreate')); 
bdclose('all'); 

% 2. LOAD DATA TREK (MAIN WORKSPACE)
if exist('Lusail_LUT.mat', 'file')
    load('Lusail_LUT.mat');
    disp('✅ Loaded "Lusail_LUT.mat" (track_dist, track_slope, track_curvature)');
else
    error('❌ File "Lusail_LUT.mat" not found! Run the Python script first to generate it.');
end

% Check if 'init_track' is still needed (Legacy support)
if exist('init_track.m', 'file') && ~exist('track_dist', 'var')
    try init_track; disp('ℹ️ Loaded legacy init_track.'); catch; end; 
end

disp('=== AI TRAINING: PARALLEL MODE WITH FULL LOGS ===');
fprintf('Mode: PARALLEL | Time Limit: %d sec\n', REAL_TIME_LIMIT);

% 3. BOUNDS (KM/H RAW)
LowerBound = [35,  25,  0.8,  25,  10]; 
UpperBound = [50,  49,  5.0,  100,  150];

% 4. SETTING GA
options = optimoptions('ga', ...
    'PopulationSize', 25, ... 
    'MaxGenerations', 40, ...
    'Display', 'iter', ...
    'UseParallel', USE_PARALLEL, ... 
    'PlotFcn', @gaplotbestf);

disp('🚀 Starting GA Loop...');

% 5. PASSING ADDITIONAL ARGS
EvaluatorFunc = @(x) RaceEvaluator(x, REAL_TIME_LIMIT);

[BestParams, BestCost] = ga(EvaluatorFunc, 5, [],[],[],[], LowerBound, UpperBound, [], options);

% =========================================================
% 6. VERIFIKASI JUARA
% =========================================================
fprintf('\n🏆 TRAINING SELESAI. VERIFIKASI JUARA...\n');

try
    simIn = Simulink.SimulationInput('Simulinkrakata_lusail');
    simIn = simIn.setModelParameter('StopTime', '600');
    simIn = simIn.setModelParameter('SimulationMode', 'normal');
    
    % Inject Params directly (Safer than assignin)
    simIn = simIn.setVariable('upper_bound', BestParams(1));
    simIn = simIn.setVariable('lower_bound', BestParams(2));
    simIn = simIn.setVariable('throttle_val', BestParams(3));
    simIn = simIn.setVariable('throttle_slew', BestParams(4));
    simIn = simIn.setVariable('coast_lookahead', BestParams(5));
    
    simOut = sim(simIn);
    
    dist = simOut.Distance.Data(end);
    ener = simOut.Energy_Result.Data(end);
    
    fprintf('JUARA: Dist %.2fm | Energy %.2f Wh | Cost %.2f\n', dist, ener, BestCost);
    if dist < 3680
        disp('⚠️ STATUS: MOGOK (DNF)');
    else
        disp('✅ STATUS: FINISH');
    end
catch ME
    fprintf('Verifikasi Gagal: %s\n', ME.message);
end

%% === BAGIAN 2: EVALUATOR (WITH REVISED PENALTY SYSTEM) ===
function Cost = RaceEvaluator(params, TimeLimit)
    % Cek Worker ID
    t = getCurrentTask();
    if isempty(t), WorkerID = 0; else, WorkerID = t.ID; end

    % === AUTO-LOAD LUT IN WORKER ===
    try
        hasMap = evalin('base', 'exist(''track_dist'', ''var'')');
        if ~hasMap
            evalin('base', 'load(''Lusail_LUT.mat'')'); 
        end
    catch
        evalin('base', 'load(''Lusail_LUT.mat'')'); 
    end

    % Decode Params
    V_up = params(1); V_low = params(2); Throt = params(3); Ramp = params(4); Look = params(5);
    
    % Logic Check
    if V_low >= V_up
        Cost = 1e12; return;
    end
    
    % Print Strategi
    fprintf('[W%d] 🏁 STRATEGY: V_up=%.1f | V_low=%.1f | Throttle=%.2f | Slew=%.1f | Look=%.1fm\n', ...
            WorkerID, V_up, V_low, Throt, Ramp, Look);

    try
        % --- SETTING SIMULINK ---
        simIn = Simulink.SimulationInput('Simulinkrakata_lusail');
        simIn = simIn.setModelParameter('SimulationMode', 'normal'); 
        simIn = simIn.setModelParameter('StopTime', '600'); 
        simIn = simIn.setModelParameter('TimeOut', TimeLimit); 
        
        % === CRITICAL FIX: USE setVariable ===
        simIn = simIn.setVariable('upper_bound', V_up);
        simIn = simIn.setVariable('lower_bound', V_low);
        simIn = simIn.setVariable('throttle_val', Throt);
        simIn = simIn.setVariable('throttle_slew', Ramp);
        simIn = simIn.setVariable('coast_lookahead', Look);

        % Run Sim
        simOut = sim(simIn); 
        
        % Check Data validity
        if isempty(simOut.Energy_Result)
             error('Data Empty (Timeout/Crash)');
        end
        
        EnergyUsed = simOut.Energy_Result.Data(end); 
        TimeTaken  = simOut.laptime.Data(end);      
        FinalDist  = simOut.Distance.Data(end);
        
        % =========================================================
        % REVISED PENALTY SYSTEM (Based on your preferences)
        % =========================================================
        Penalty = 0;
        Status = 'FINISH';
        
        % 1. DNF Penalty (Primary - Harsh)
        if FinalDist < 3680
            Penalty = 1e9 + (3680 - FinalDist) * 1000; 
            Status = 'MOGOK (DNF)';
            
        % 2. Time Window Penalty (400-500 seconds range)
        elseif TimeTaken < 400
            % Too fast penalty
            Penalty = (400 - TimeTaken) * 20000;
            Status = 'TERLALU CEPAT';
        elseif TimeTaken > 500
            % Too slow penalty
            Penalty = (TimeTaken - 500) * 20000;
            Status = 'TERLALU LAMBAT';
        end
        
        % 3. Braking Penalty (Gentle - as requested)
        PenaltyBrake = 0;
        if isprop(simOut, 'Brake_Log')
            BrakeSum = sum(simOut.Brake_Log.Data);
            PenaltyBrake = BrakeSum * 0.0001; % Gentle multiplier
            if BrakeSum > 0 && ~strcmp(Status, 'MOGOK (DNF)')
                Status = [Status ' + BRAKE'];
            end
        end
        
        % Final Cost = Energy + Penalties
        Cost = EnergyUsed + Penalty + PenaltyBrake;
        
        % =========================================================
        % LOGGING HASIL
        % =========================================================
        if contains(Status, 'MOGOK')
            % DNF Case
            fprintf('[W%d] ❌ %s (%.1fm) | 🔋 %.2f Wh | Penalty: %.1e | Total Cost: %.1e\n', ...
                WorkerID, Status, FinalDist, EnergyUsed, Penalty + PenaltyBrake, Cost);
        else
            % Finish Case (with time info)
            fprintf('[W%d] ✅ %s | ⏱️ %.1fs | 🔋 %.2f Wh | Penalty: %.1f | Total Cost: %.1f\n', ...
                WorkerID, Status, TimeTaken, EnergyUsed, Penalty + PenaltyBrake, Cost);
        end
        
    catch ME
        % Better Error Reporting
        errMsg = ME.message;
        if ~isempty(ME.cause)
            errMsg = ME.cause{1}.message;
        end
        fprintf('\n[W%d] 🔴 CRASH: %s\n', WorkerID, errMsg);
        Cost = 1e12; 
    end
end