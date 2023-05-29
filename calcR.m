function R = calcR(rarr,barr,N)
rb = mean(rarr); bb = mean(barr);
R = zeros(2);
for i = 1:N
    r = rarr(i); b = barr(i);
    R = R + ...
        [(r-rb)^2,        (r-rb)*(b-bb);
         (b-bb)*(r-rb),   (b-bb)^2];
end
R = R / (N-1);
end

