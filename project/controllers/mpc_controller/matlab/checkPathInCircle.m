function y = checkPathInCircle(posMatrix, H, sampleIndex)
    for i = 1:H
        if ~(posMatrix(1,i,sampleIndex)^2 + posMatrix(2,i,sampleIndex)^2 < 25 && ...
                posMatrix(1,i,sampleIndex)^2 + posMatrix(2,i,sampleIndex)^2 > 0)
            y = false;
            return
        end

    end
    y = true;

end