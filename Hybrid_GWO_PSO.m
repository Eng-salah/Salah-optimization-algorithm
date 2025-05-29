%%
% * Author:  Eng. Salah Abbas Taha (S.A.Taha)(salahabbas041@gmail,com,07748191514,Tele.@Eng_441)
% * Date: 06.11.2024
function D = Hybrid_GWO_PSO(Vpv, Ipv,Enabled)
Dinit = 0.5;
SearchAgents_no=6;   % Number of search agents
Max_iteration=10;    % Maximum numbef of iterations
lb = 0.3;            % lb is the lower bound: lb=[lb_1,lb_2,...,lb_d]
ub = 0.9;            % up is the uppper bound: ub=[ub_1,ub_2,...,ub_d]
w=0.5;
a=2;
power_delta =0.01;


% initialize alpha, beta, and delta_pos
persistent params;

if isempty(params)
    params = initialize(SearchAgents_no,lb,ub); 
end



    
% Calculate power
fitness = Vpv * Ipv; 

% Weather conditions test 
if params.runing==false
    
    if abs(fitness - params.power_last) / params.power_last >= power_delta
        params = initialize(SearchAgents_no,lb,ub); 
    else
        D = params.Alpha_pos;
        params.power_last = fitness;
        return;
    end
        
end

params.power_last = fitness;
params.fitness(params.index) = fitness; % Store fitness for the current wolf

% Output duty cycle for the current wolf
D = params.Positions(params.index);

% Step 2: Check if all wolves have been evaluated
params.index = params.index + 1;



if params.index > SearchAgents_no
    
    % Reset index after evaluating all wolves
    params.index = 1; 
    params.iter = params.iter + 1;
    
    % Update dominant levels (alpha, beta, delta)
    [sorted_fitness, sorted_idx] = sort(params.fitness, 'descend');
    params.Alpha_score = sorted_fitness(1); 
    params.Alpha_pos = params.Positions(sorted_idx(1));
    params.Beta_score = sorted_fitness(2); 
    params.Beta_pos = params.Positions(sorted_idx(2));
    params.Delta_score = sorted_fitness(3); 
    params.Delta_pos = params.Positions(sorted_idx(3));
    
    
   
    
    
    for i=1:size(params.Positions,1)
        
        r1=rand(); % r1 is a random number in [0,1]
        r2=rand(); % r2 is a random number in [0,1]
        r3=rand(); % r3 is a random number in [0,1]
        
        A1=2*a*r1-a; 
        C1=2*r2; 
        D_alpha=abs(C1*params.Alpha_pos-w*params.Positions(i)); 
        X1=params.Alpha_pos-A1*D_alpha; 

  
        A2=2*a*r1-a; 
        C2=2*r2; 
        D_beta=abs(C2*params.Beta_pos-w*params.Positions(i)); 
        X2=params.Beta_pos-A2*D_beta; 

       
        A3=2*a*r1-a; 
        C3=2*r2;
        D_delta=abs(C3*params.Delta_pos-w*params.Positions(i)); 
        X3=params.Delta_pos-A3*D_delta; 

        % velocity updation
        params.velocity(i)=w*(params.velocity(i)+C1*r1*(X1-params.Positions(i))+C2*r2*(X2-params.Positions(i))+C3*r3*(X3-params.Positions(i)));
        % positions update
        newPosition = params.Positions(i)+params.velocity(i);
        
        % Enforce boundaries on duty cycle
        if newPosition > ub
            newPosition = ub;
        elseif newPosition < lb
            newPosition = lb;
        end
        
        params.Positions(i)=newPosition;
                   
    end%end for.
    
    
    % Check termination conditions
    if params.iter >= Max_iteration 
%         fprintf('Converged or Max Iterations Reached\n');
        params.runing = false;
        D = params.Alpha_pos;
    end
 
    
end%end if.

if Enabled==0
    D = Dinit;
end


end%end function.


function params = initialize(SearchAgents_no,lb,ub)

params.Alpha_pos= 0;
params.Alpha_score = 0; 

params.Beta_pos=0;
params.Beta_score=0; 
params.Delta_pos=0;
params.Delta_score=0;

%Initialize the positions of search agents
params.Positions= rand(SearchAgents_no,1).*(ub-lb)+lb;
params.fitness = zeros(1,SearchAgents_no); % Fitness values (power values)
params.velocity = zeros(SearchAgents_no,1) ;%0.3*randn(SearchAgents_no,1) ;
params.iter=0;
params.index = 1;
params.runing = true;
params.power_last =0;

end%end function.
