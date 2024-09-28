function sigma = get_sigma(t)
    s_list = [1 3 2 2 4 1 4 3 2];
    %s_list = [4 4 4 4 4 4 4 4 4];
    global m;
    sigma = 1;
    i = 1;
    attacktime = [2 3;
                  5 6;   
                  10 11;
                  15 16;
                  17 20;
                  25 28;
                  30 33;
                  36 42;
                  45 47];
    for i = 1:9
    if ((t >= attacktime(i,1)) && (t < attacktime(i,2)))
        sigma = s_list(i);
    end
    end
end
