function H = hankel_c(w,t1,t2,r)
    
    % Generates Hankel matrix in a C-code compatible way

    H = zeros(r*t1, t2);
    for idx = 1:t2
        H(:,idx) = w(1+(idx-1)*r : (idx-1)*r + r*t1);
    end
end