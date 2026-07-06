function u = generatePWM(t, A, pulseLength)
% Input:
% t = time length
% A = amplitude of PWM signal
% pulseLength = duty cycle

% Output: 
%u = PWM signal

    u = zeros(size(t));

    cycleLength = 12;     
    pulseStarts = [0 3 6 9];
    pulseSigns  = [1 1 -1 -1];
    nCycles = ceil(max(t)/cycleLength);

    for k = 0:nCycles-1

        cycleOffset = k * cycleLength;

        for i = 1:length(pulseStarts)

            startTime = cycleOffset + pulseStarts(i);
            endTime   = startTime + pulseLength;

            u(t >= startTime & t < endTime) = ...
                pulseSigns(i) * A;

        end
    end

end