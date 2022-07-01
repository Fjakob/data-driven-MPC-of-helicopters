function H = hankel_r(w,t1,t2,r) 
% r: number of components of w
% works only for SISO systems
% w = [1;1;1;2;2;2;3;3;3;4;4;4;5;5;5;6;6;6;7;7;7;8;8;8;9;9;9;10;10;10];
% t1 = 4;
% t2 = 10-4+1;
x = [ w(1:r*(t1+t2-1)); flipud(w(1:r*(t1+t2-r))) ];       % build vector of user data
ij = (1:r*t1)' + (0:r:r*(t2-1));           % Hankel subscripts
H = x(ij);                         % actual data
if isrow(ij)                       % preserve shape for a single row
    H = H.';
end