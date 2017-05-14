function obj = propagate_particles(obj)
% PURPOSE : Propagate particles for each target
% INPUT : - obj = @TrackerBPF object
% OUTPUT : - obj = updated @TrackerBPF object
% AUTHORS: Kenji Okuma
% DATE: January 2007
% =========================================================================

targets = obj.targets;

i = 1;
num_pc = targets{i}.num_pc;
while ~isemptycell(targets(i)) && i <= obj.max_num_targets
    if targets{i}.status
        particlePred = zeros(size(targets{i}.particle));
        if targets{i}.num_ada == 0 % non BPF
            noise = randnorm(num_pc, [0; 0; 0], [], obj.prior_noise_cov);
            noise(3, :) = noise(3, :) / obj.box_width; % box_width = box_height
            % update particles with two dynamics
            for j = 1 : num_pc
                if rand > 0.5
                    % Dynamics option: Simply gaussian noise
                    particlePred(:,j) = targets{i}.particle(:,j) + noise(:, j);
                else
                    % Dynamics: Second order auto regression with constant
                    % velocity
                    particlePred(:,j) = 2*targets{i}.particle(:,j) - targets{i}.particlePast(:,j) + noise(:,j);
                end                
                % enforce the minimum scale of the target
                particlePred(3,j) = max(0.3, particlePred(3,j));
            end

            targets{i}.x_prior_mu = particlePred - noise;

        else % BPF case
            noise = randnorm(num_pc, [0; 0; 0], [], obj.prior_noise_cov);
            noise(3, :) = noise(3, :) / obj.box_width; % box_width = box_height
            for j = 1:num_pc
                % Do not apply second order dynamics to boosted particles
                if rand > 0.5 
                    % Dynamics option: Simply gaussian noise
                    particlePred(:, j) = targets{i}.particle(:, j) + noise(:, j);
                else
                    % Dynamics: Second order auto regression with constant
                    % velocity
                    particlePred(:, j) = 2*targets{i}.particle(:, j) - targets{i}.particlePast(:, j) + noise(:, j);                    
                end
                % enforce the minimum scale of the target
                particlePred(3,j) =  max(0.3, particlePred(3,j));   
            end
            
            targets{i}.x_prior_mu = particlePred - noise;
            targets{i}.boost_mu = targets{i}.x_prior_mu;
 
        end

        targets{i}.x_prior_cov = obj.prior_noise_cov;
		targets{i}.boost_cov = targets{i}.x_prior_cov;
				
        targets{i}.particlePast = targets{i}.particle;
        targets{i}.particle = particlePred;				

        % Check if the particles are dead or not
        targets{i}.pcstatus = checkstatus(obj, targets{i}.particle);
    end
    i = i + 1;
end

obj.targets = targets;
