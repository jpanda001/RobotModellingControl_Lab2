function myrobot = mypuma560(DH)

    links = [Link];   
    DHsize = size(DH);
    for i = 1:DHsize(1)
         links(i) = Link('d',DH(i,2), 'a', DH(i,3), 'alpha', DH(i,4));
    end
   
    myrobot = SerialLink(links,'name', 'puma560');
    
end