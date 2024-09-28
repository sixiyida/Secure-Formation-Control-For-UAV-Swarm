function sigma = get_sigma(t)
    global m;
    gap = 5;
    sigma = mod(floor(t / gap), 3) + 1; 
end
