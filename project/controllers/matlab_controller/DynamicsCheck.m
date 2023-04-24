function y = DynamicsCheck(H, posMatrix, idx, dt)
    for i  = 1:H
        if (posMatrix(1,i+1,idx) - posMatrix(1, i , idx) - dt * posMatrix(4, i, idx)  * cos(posMatrix(3,i,idx))  > 1e-4)
            y = false;
            return;
        end 
        if (posMatrix(2,i+1,idx) - posMatrix(2, i , idx) - dt * posMatrix(4, i, idx) * sin(posMatrix(3,i,idx))  >  1e-4)
            y = false;
            return;
        end 
        if (posMatrix(3, i+1 , idx) - posMatrix(3, i, idx) - dt * posMatrix(5, i+1, idx) >  1e-4)
            y = false;
            return;
        end 
        if (posMatrix(4, i+1, idx) - posMatrix(4, i, idx) - dt * posMatrix(6, i+1, idx) >  1e-4)
            y = false;
            return;
        end 
            
    end
    y = true;
end