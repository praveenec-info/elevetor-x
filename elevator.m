clc;
clear;
close all;

%% SYSTEM PARAMETERS
numFloors = 10;
numElevators = 3;
maxWeightKg = 600;          % Maximum load per elevator (kg)
simTime = 500;
travelTime = 3;
doorTime = 2;

rng(42);

%% GENERATE PASSENGER REQUESTS
arrivalTimes = sort(randi([1 simTime], 1, 80));
numPassengers = length(arrivalTimes);

origin = randi([1 numFloors], 1, numPassengers);
destination = randi([1 numFloors], 1, numPassengers);

% Give each passenger a random weight between 35 kg and 100 kg
passengerWeightKg = randi([35 100], 1, numPassengers);

for i = 1:numPassengers
    while destination(i) == origin(i)
        destination(i) = randi([1 numFloors]);
    end
end

%% RUN BOTH ALGORITHMS
% Algorithm 1: Nearest elevator
% Algorithm 2: Optimized elevator scheduling

results = cell(1,2);
algorithmNames = {'Nearest Lift', 'Optimized Scheduling'};

for alg = 1:2

    floorPos = ones(1, numElevators);
    direction = zeros(1, numElevators);
    loadWeightKg = zeros(1, numElevators);
    moveClock = zeros(1, numElevators);
    dwell = zeros(1, numElevators);

    onboard = cell(1, numElevators);
    assigned = cell(1, numElevators);

    for e = 1:numElevators
        onboard{e} = [];
        assigned{e} = [];
    end

    status = zeros(1, numPassengers);
    pickupTime = nan(1, numPassengers);
    dropTime = nan(1, numPassengers);

    totalTravel = 0;
    totalStops = 0;

    for t = 1:simTime

        %% ASSIGN NEW REQUESTS
        newRequests = find(arrivalTimes == t);

        for p = newRequests

            scores = inf(1, numElevators);

            for e = 1:numElevators

                % Count onboard and assigned passengers in reserved weight.
                reservedPassengers = [onboard{e}, assigned{e}];
                reservedWeightKg = sum(passengerWeightKg(reservedPassengers));

                if reservedWeightKg + passengerWeightKg(p) > maxWeightKg
                    continue;
                end

                dist = abs(floorPos(e) - origin(p));
                scores(e) = dist * travelTime;

                if alg == 2
                    if direction(e) ~= 0
                        reqDir = sign(destination(p)-origin(p));

                        if direction(e) ~= reqDir
                            scores(e) = scores(e) + 8;
                        end
                    end

                    % Penalize a heavily loaded elevator.
                    scores(e) = scores(e) + ...
                                2*length(assigned{e}) + ...
                                1.5*(loadWeightKg(e)/75);
                end
            end

            [bestScore, bestElevator] = min(scores);

            if isfinite(bestScore)
                assigned{bestElevator}(end+1) = p;
                status(p) = 1;
            end
        end

        %% MOVE ELEVATORS AND SERVE REQUESTS
        for e = 1:numElevators

            % Drop passengers at their destination
            if ~isempty(onboard{e})
                dropList = onboard{e}( ...
                    destination(onboard{e}) == floorPos(e));

                for p = dropList
                    status(p) = 3;
                    dropTime(p) = t;
                    totalStops = totalStops + 1;
                end

                onboard{e}( ...
                    destination(onboard{e}) == floorPos(e)) = [];
            end

            % Pick up passengers only when the weight limit allows it.
            if ~isempty(assigned{e})
                pickupList = assigned{e}( ...
                    origin(assigned{e}) == floorPos(e));

                currentLoadKg = sum(passengerWeightKg(onboard{e}));
                pickedNow = [];

                for p = pickupList
                    if currentLoadKg + passengerWeightKg(p) <= maxWeightKg
                        status(p) = 2;
                        pickupTime(p) = t;
                        onboard{e}(end+1) = p;
                        currentLoadKg = currentLoadKg + passengerWeightKg(p);
                        pickedNow(end+1) = p;
                        totalStops = totalStops + 1;
                    end
                end

                assigned{e}(ismember(assigned{e}, pickedNow)) = [];
            end

            loadWeightKg(e) = sum(passengerWeightKg(onboard{e}));

            if dwell(e) > 0
                dwell(e) = dwell(e) - 1;
                continue;
            end

            % Determine the next target floor
            targets = [];

            if ~isempty(onboard{e})
                targets = [targets destination(onboard{e})];
            end

            if ~isempty(assigned{e})
                targets = [targets origin(assigned{e})];
            end

            if isempty(targets)
                direction(e) = 0;
                continue;
            end

            % Prefer continuing in the current direction
            if direction(e) > 0
                ahead = targets(targets > floorPos(e));

                if ~isempty(ahead)
                    target = min(ahead);
                else
                    target = max(targets);
                end

            elseif direction(e) < 0
                ahead = targets(targets < floorPos(e));

                if ~isempty(ahead)
                    target = max(ahead);
                else
                    target = min(targets);
                end

            else
                [~, idx] = min(abs(targets-floorPos(e)));
                target = targets(idx);
            end

            if target ~= floorPos(e)
                direction(e) = sign(target-floorPos(e));
                moveClock(e) = moveClock(e) + 1;

                if moveClock(e) >= travelTime
                    floorPos(e) = floorPos(e) + direction(e);
                    moveClock(e) = 0;
                    totalTravel = totalTravel + 1;
                end
            else
                dwell(e) = doorTime;
            end
        end
    end

    %% CALCULATE PERFORMANCE
    completed = find(status == 3);

    if ~isempty(completed)
        wait = pickupTime(completed) - arrivalTimes(completed);
        journey = dropTime(completed) - pickupTime(completed);

        results{alg}.avgWait = mean(wait);
        results{alg}.maxWait = max(wait);
        results{alg}.avgJourney = mean(journey);
    else
        results{alg}.avgWait = NaN;
        results{alg}.maxWait = NaN;
        results{alg}.avgJourney = NaN;
    end

    results{alg}.completed = length(completed);
    results{alg}.total = numPassengers;
    results{alg}.travel = totalTravel;
    results{alg}.stops = totalStops;

    fprintf('\n%s\n', algorithmNames{alg});
    fprintf('Completed passengers: %d / %d\n', ...
        length(completed), numPassengers);
    fprintf('Average waiting time: %.2f seconds\n', ...
        results{alg}.avgWait);
    fprintf('Maximum waiting time: %.2f seconds\n', ...
        results{alg}.maxWait);
    fprintf('Average journey time: %.2f seconds\n', ...
        results{alg}.avgJourney);
    fprintf('Total floor movements: %d\n', totalTravel);
end

%% PERFORMANCE GRAPHS
avgWait = [results{1}.avgWait results{2}.avgWait];
maxWait = [results{1}.maxWait results{2}.maxWait];
avgJourney = [results{1}.avgJourney results{2}.avgJourney];
travel = [results{1}.travel results{2}.travel];

figure('Name','Elevator Scheduling Comparison');

subplot(2,2,1);
bar(avgWait);
set(gca,'XTickLabel',algorithmNames);
ylabel('Seconds');
title('Average Waiting Time');
grid on;

subplot(2,2,2);
bar(maxWait);
set(gca,'XTickLabel',algorithmNames);
ylabel('Seconds');
title('Maximum Waiting Time');
grid on;

subplot(2,2,3);
bar(avgJourney);
set(gca,'XTickLabel',algorithmNames);
ylabel('Seconds');
title('Average Journey Time');
grid on;

subplot(2,2,4);
bar(travel);
set(gca,'XTickLabel',algorithmNames);
ylabel('Floor movements');
title('Total Elevator Travel');
grid on;

sgtitle('Elevator Scheduling Performance');

%% WAITING TIME IMPROVEMENT
if avgWait(1) > 0
    improvement = (avgWait(1)-avgWait(2))/avgWait(1)*100;
    fprintf('\nWaiting time improvement: %.2f%%\n', improvement);
end

