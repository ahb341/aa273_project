function Q = calcQ(xarr,yarr,tharr,N)
xb = mean(xarr); yb = mean(yarr); thb = mean(tharr);
Q = zeros(3);
for i = 1:N
    x = xarr(i); y = yarr(i); th = tharr(i);
    Q = Q + ...
        [(x-xb)^2,        (x-xb)*(y-yb),    (x-xb)*(th-thb);
         (y-yb)*(x-xb),   (y-yb)^2,         (y-yb)*(th-thb);
         (th-thb)*(x-xb), (th-thb)*(y-yb),  (th-thb)^2];
end
Q = Q / (N-1);
end

