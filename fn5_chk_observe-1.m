%% Observability 

function obsv_matrix = chk_observe(A,C)
obsv_matrix=obsv(A,C); 
obsv_rank = rank(obsv_matrix);   
if(obsv_rank < length(A))     
    fprintf('System is not observable and the rank of observability matrix is %d.\n',obsv_rank) 
else
    fprintf('System is observable and the rank of observability matrix is %d.\n',obsv_rank) 
end