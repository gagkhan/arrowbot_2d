function y = mapangle(ang)
% To map angles 'ang' to (-180,180]
y = mod(ang,2*pi);
temp = y > pi;
y(temp) = y(temp) - 2*pi;
end