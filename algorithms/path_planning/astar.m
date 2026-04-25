function [path] = astar(read_only_vars, public_vars)
%ASTAR Summary of this function goes here

path = [];

goal = round(read_only_vars.discrete_map.goal);
start = round(public_vars.estimated_pose(1:2) *5 ); %*5 pro prepocet na meritko matice
occupancy_grid = read_only_vars.discrete_map.map;

% 8 smerů možného pohybu
move_directions = [0 1; 1 1; 1 0; 1 -1; 0 -1; -1 -1; -1 0; -1 1];



[rows, cols] = size(occupancy_grid);
g_cost = inf(rows, cols);      % cena cesty
f_cost = inf(rows, cols);      % cena do cíle g+h
parent_x = zeros(rows, cols);  % rodič (z jakeho bodu se dostal do aktualniho)
parent_y = zeros(rows, cols);  

% start
g_cost(start(2), start(1)) = 0;
f_cost(start(2), start(1)) = norm(start - goal);

open_list = [start(1), start(2)]; % start jako první list


path_found = false;


% dilatace zdí pomocí konvoluce:
kernel = ones(5,5); % jádro 3x3 roztáhne překážku o jeden pixel na všechny strany (5*5 o 2 pixely atd.)
dilated_grid = conv2(occupancy_grid, kernel, 'same'); %same pro oříznutí na původní velikost
occupancy_grid = dilated_grid > 0; % převod zpět na 1 a 0 (cokoli > 0 je teď zeď)



% prohledávací smyčka
while ~isempty(open_list)
    best_idx = 1;
    min_f_cost = inf;

    % hledání nejmenší ceny do cíle
    for i = 1:size(open_list, 1)
        x = open_list(i,1);
        y = open_list(i,2);
        if f_cost(y, x) < min_f_cost
            min_f_cost = f_cost(y, x);
            best_idx = i;
        end
    end

    point = open_list(best_idx, :);

    % odpovidá bod cíly
    if point(1) == goal(1) && point(2) == goal(2)
        path_found = true;
        break; 
    end

    open_list(best_idx, :) = [];


    % prozkoumání sousedů
    for i = 1:size(move_directions,1)
        neighbor_x = point(1) + move_directions(i,1);
        neighbor_y = point(2) + move_directions(i,2);

        %oveření platnosti souseda (je v mapě a není to zeď)
        if neighbor_x >= 1 && neighbor_y >= 1 && neighbor_x <= cols && neighbor_y <= rows && occupancy_grid(neighbor_y, neighbor_x) == 0

            move_cost = norm(move_directions(i,:));
            new_g_cost = g_cost(point(2), point(1)) + move_cost;

            % aktializovat souseda, pokud je nelezena cesta s nižší cenou
            if new_g_cost < g_cost(neighbor_y, neighbor_x)
                g_cost(neighbor_y, neighbor_x) = new_g_cost;
                f_cost(neighbor_y, neighbor_x) = new_g_cost + norm([neighbor_x neighbor_y] - goal);

                parent_x(neighbor_y, neighbor_x) = point(1);
                parent_y(neighbor_y, neighbor_x) = point(2);



                % přidání souseda do listů pokud tam ještě není
                in_list = false;
                for j = 1:size(open_list, 1)
                    if open_list(j,1) == neighbor_x && open_list(j,2) == neighbor_y
                        in_list = true; 
                        break;
                    end
                end
                if ~in_list
                    open_list = [open_list; neighbor_x, neighbor_y]; 
                end
            end
        end
    end
end




if path_found
    current_point = goal;
    while current_point(1) ~= 0 && current_point(2) ~= 0 % dokud nenajmeme start (rodič 0 0)
        path = [current_point - 1; path]; % přidání bodu na začátek cesty

        % Najdeme rodiče aktuálního bodu
        curr_parent_x = parent_x(current_point(2), current_point(1));
        curr_parent_y = parent_y(current_point(2), current_point(1));
        current_point = [curr_parent_x, curr_parent_y];
    end

    path = path./5;

else
    path = [];
    fprintf('No path found. \n');
end



end
