%% random sampled MPC

function y = RMPC(H, numberOfRandomSamples, xinit, goalX, goalY,  centerX, centerY, innerR, outerR)
    pos = [xinit;0;0];
    dt = 0.01;
    numberOfStates = 4; 
    numberOfControls = 2;
    posMatrix = zeros(6, H+1, numberOfRandomSamples);
    for i = 1:numberOfRandomSamples
        posMatrix(:, 1, i) = pos';
    end
    
    w_bound = [-2 * pi 2 * pi];
    a_bound = [-20 20];

    % rand number between interval (a,b) r = a + (b-a) *rand(Number of
    w_rand = w_bound(1) + (w_bound(2) - w_bound(1)) * rand(numberOfRandomSamples,1);
    a_rand = a_bound(1) + (a_bound(2) - a_bound(1)) * rand(numberOfRandomSamples,1);
    
    % calculate the next position 
    for k = 1:H
        w_rand = w_bound(1) + (w_bound(2) - w_bound(1)) * rand(numberOfRandomSamples,1);
        a_rand = a_bound(1) + (a_bound(2) - a_bound(1)) * rand(numberOfRandomSamples,1);
        for i = 1:numberOfRandomSamples
            posMatrix(1, k+1, i) = posMatrix(1,k,i) + dt * posMatrix(4, k,i) * cos(posMatrix(3, k,i));
            posMatrix(2, k+1, i) = posMatrix(2,k,i) + dt * posMatrix(4, k,i) * sin(posMatrix(3, k,i));
            posMatrix(3,k+1,i) = posMatrix(3,k,i) + dt * w_rand(i);
            posMatrix(4,k+1,i) = posMatrix(4, k, i) + dt * a_rand(i);
            posMatrix(5,k+1,i) = w_rand(i);
            posMatrix(6,k+1,i) = a_rand(i);
        end
    end
%     norm(posMatrix(1,:,i)) + norm(posMatrix(2,:,i)) <= 100 ...
%                 && norm(posMatrix(1,:,i)) + norm(posMatrix(2,:,i)) >= 0 ...
    validPaths = [];
    for i = 1:numberOfRandomSamples
        if (    DynamicsCheck(H, posMatrix, i, dt) && ...
                checkPathInCircle(posMatrix, H, i, centerX(1), centerY(1), innerR, outerR) ... % checkPathInCircle(posMatrix,H, i)
                && all(posMatrix(1,:,i) < 1000) && all(posMatrix(1,:,i) > -1000) ...
                && all(posMatrix(3,:,i) > -2* pi) && all(posMatrix(3,:,i) < 2 * pi) ...
                && all(posMatrix(4,:,i) > -500) && all(posMatrix(4,:,i) < 1000) ...
                && all(posMatrix(2,:, i) > -1000) && all(posMatrix(2,:,i) < 1000))
            validPaths = [validPaths i];
        end
    end 

    minCost = 1e100;
    minCostIndex = -100;
    for i = 1:length(validPaths)
        %cost = -sum(posMatrix(4,:,validPaths(i)));
        %cost = sum((posMatrix(1,:,validPaths(i)) - goalX).^2) + sum((posMatrix(2,:,validPaths(i)) - goalY).^2); %- sum(abs(posMatrix(4,:,validPaths(i)))); % sum(abs((posMatrix(1, :, i) - centerX(1)).^2 + (posMatrix(2, :, i) - centerY(1)).^2 )) ^2
        % (x_n - x_r)^2 * Q_n + sum(x_k - x_r)^2 + u_k^2 * R)   
        cost =  sum( - 2* posMatrix(4,:,validPaths(i)) + 0.75 * abs((posMatrix(1, :, i) - centerX(1)).^2 + (posMatrix(2, :, i) - centerY(1)).^2 -  30^2));
        if (cost < minCost)
            minCost = cost;
            minCostIndex = validPaths(i);
        end
    end
    if (minCostIndex == -100)
        y = [-999999];
    else
        % CHECK THE ROAD CONSTRAINTS    
        control_w = posMatrix(5,2,minCostIndex);
        control_v = posMatrix(6,2,minCostIndex);
        minCost;
        y = [control_w control_v];
    end
end

