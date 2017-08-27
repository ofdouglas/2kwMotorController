function p = pos_from_velocity(v)
%integrator = tf(1, [1 0]);
%p = conv(v, impulse(integrator));
    
p = zeros(1,length(v));
accum = 0;
for i = 1:length(p)
    accum = accum + v(i);
    p(i) = accum;
end
