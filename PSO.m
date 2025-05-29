%%
% * Author:  Eng. Salah Abbas Taha (S.A.Taha)(salahabbas041@gmail,com,07748191514,Tele.@Eng_441)
% * Date: 06.11.2024
function D = PSO(Vpv, Ipv, Enabled)
    % Initial duty cycle
    Dinit = 0.5;
    % Number of particles
    SearchAgents_no = 6;   
    % Maximum number of iterations
    Max_iteration = 10;    
    % Lower bound
    lb = 0.3;              
    % Upper bound
    ub = 0.9;              
    % Inertia weight
    w = 0.4;
    % Cognitive and social coefficients
    c1 = 1.6;
    c2 = 1.2;
    % Power change threshold
    power_delta = 0.01;

    % Initialize particles
    persistent params;

    if isempty(params)
        params = initialize(SearchAgents_no, lb, ub); 
    end

    % Calculate fitness (power)
    fitness = Vpv * Ipv; 

    % Check weather conditions
    if params.running == false
        if abs(fitness - params.power_last) / params.power_last >= power_delta
            params = initialize(SearchAgents_no, lb, ub); 
        else
            D = params.Gbest_pos;
            params.power_last = fitness;
            return;
        end
    end

    params.power_last = fitness;
    params.fitness(params.index) = fitness; % Store fitness for the current particle

    % Update personal best position if current fitness is better
    if fitness > params.pbest_fitness(params.index)
        params.pbest_fitness(params.index) = fitness;
        params.pbest_pos(params.index) = params.Positions(params.index);
    end

    % Output duty cycle for the current particle
    D = params.Positions(params.index);

    % Check if all particles have been evaluated
    params.index = params.index + 1;

    if params.index > SearchAgents_no
        % Reset index after evaluating all particles
        params.index = 1; 
        params.iter = params.iter + 1;

        % Update global best position
        [~, best_idx] = max(params.fitness);
        params.Gbest_pos = params.Positions(best_idx);

        for i = 1:size(params.Positions, 1)
            r1 = rand();
            r2 = rand();
            
            % Update velocity
            params.velocity(i) = w * params.velocity(i) ...
                                 + c1 * r1 * (params.pbest_pos(i) - params.Positions(i)) ...
                                 + c2 * r2 * (params.Gbest_pos - params.Positions(i));
            % Update position
            newPosition = params.Positions(i) + params.velocity(i);

            % Enforce boundaries
            if newPosition > ub
                newPosition = ub;
            elseif newPosition < lb
                newPosition = lb;
            end
            
            params.Positions(i) = newPosition;
        end

        % Termination conditions
        if params.iter >= Max_iteration 
            params.running = false;
            D = params.Gbest_pos;
        end
    end

    if Enabled == 0
        D = Dinit;
    end
end

function params = initialize(SearchAgents_no, lb, ub)
    % Initialize particle Positions
    p = rand(SearchAgents_no, 1) .* (ub - lb) + lb;
    params.Positions = p;
    % Initialize fitness values
    params.fitness = zeros(1, SearchAgents_no);
    % Initialize velocities
    params.velocity = zeros(SearchAgents_no, 1);
    % Initialize iteration counter
    params.iter = 0;
    % Initialize index
    params.index = 1;
    % Set running flag
    params.running = true;
    % Initialize last power value
    params.power_last = 0;
    % Initialize personal best Positions
    params.pbest_pos = p;
    % Initialize personal best fitness values
    params.pbest_fitness = -inf(1, SearchAgents_no); 
    % Initialize global best position
    params.Gbest_pos = p(1);
end