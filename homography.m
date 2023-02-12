function H = homography(X,Y,x,y)
    % Code Reference: https://www.mathworks.com/matlabcentral/answers/26141-homography-matrix
        A = zeros(length(x(:))*2,9);
    
        for i = 1:length(x(:))
            a = [x(i),y(i),1];
            A((i-1)*2+1:(i-1)*2+2,1:9) = [[[x(i),y(i),1] [0 0 0];[0 0 0] [x(i),y(i),1]] -[X(i);Y(i)]*a];
        end
    
        [U,~,V] = svd(A);
        H = reshape(V(:,9),3,3)';
    end  