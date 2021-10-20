%% function that defines the puma560 robot using DH parameters
function structure = mypuma560(DH)
    % Creating a Link object for each robot joint
    for i = 1:length(DH)
        L(i) = Link(DH(i, :), 'standard');
    end
    
    % Creating robot structure using SerialLink
    structure = SerialLink(L, 'name', 'puma560');
end