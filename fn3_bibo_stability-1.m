%% BIBO stability

function fn3_bibo_stability(pole)   
if pole < 0     
    fprintf('The system is BIBO stable.\n') 
else
    fprintf('The system is not BIBO stable.\n') 
end


