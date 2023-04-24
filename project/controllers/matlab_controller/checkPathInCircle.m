function y = checkPathInCircle(posMatrix, H, sampleIndex, centerX, centerY, innerRadius, outerRadius)
    for i = 1:H+1
        if ~((posMatrix(1,i,sampleIndex) - centerX )^2 + (posMatrix(2,i,sampleIndex) - centerY )^2 <= outerRadius^2 && ...
                (posMatrix(1,i,sampleIndex) - centerX) ^2 + (posMatrix(2,i,sampleIndex) - centerY )^2 >= innerRadius^2)
            y = false;
            return
        end

    end
    y = true;

end