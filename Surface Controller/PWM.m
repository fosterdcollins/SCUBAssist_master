function [ PWM ] = PWM(Force)
Force;

for i = 1:length(Force)
    if(Force(i) > 0)
        %positive
        %disp('pos')
        PWM(i) = 122.33*Force(i)^(2/3)+1499;
        
    elseif Force(i) == 0
        PWM(i) = 1500;
        
    else
        %negative
        %disp('neg')
        PWM(i) = 157.32*-abs(Force(i))^(2/3)+1484;
    end
end

end

