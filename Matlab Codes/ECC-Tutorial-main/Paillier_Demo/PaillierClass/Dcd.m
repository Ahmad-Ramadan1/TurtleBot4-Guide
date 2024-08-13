function [x,u] = Dcd(z,s,q)
    arguments
        z
        s {mustBePositive}
        q {mustBeModulus(q)}
    end
    u = floor(z/q+vpa(1/2));
    y = z-u*q;
    x = y/s;
end