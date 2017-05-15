%evalFunc calculates the error given a k
%find4me controls that the error is equal to the desired one

function k = find4me(options, kStart, epsilon)
    %optimopt = optimset('Display','iter');
    f = @(x) evalFunc(x, options) - epsilon;
    k = fzero(f, kStart);
end

function e = evalFunc(k, opts)
    opts.penalizationFactor = k;
    qStar = invk(opts.robot, opts);
    e = norm(opts.xStop - getEEpos(opts.robot, qStar));
end