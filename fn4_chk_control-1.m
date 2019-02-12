%%  Controllability  
function ctrb_matrix = chk_control(A,B)
ctrb_matrix = ctrb(A,B); 
ctrb_rank = rank(ctrb_matrix);   
if(ctrb_rank < length(A))     
    fprintf('System is not controllableand the rank of controllability matrix is %d.\n',ctrb_rank)  
else
    fprintf('System is controllable and the rank of controllability matrix is %d.\n',ctrb_rank)
end

