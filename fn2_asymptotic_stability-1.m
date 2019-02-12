%% Asymptotic stability

function fn2_asymptotic_stability(eigenvalues) 
m=0;
for count=1:size(eigenvalues)
if eigenvalues(count) >= 0     
  m=m+1;  
end
end
if m>0
fprintf('The system is not asymptotically stable.\n') 
else
    fprintf('The system is asymptotically stable.\n') 
end