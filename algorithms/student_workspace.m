function [public_vars] = student_workspace(read_only_vars, public_vars)

% ---  Inicializace ---
if (read_only_vars.counter == 1)
    public_vars.state = "INITIALIZING"; % Výchozí stav
    public_vars.init_counter = 0;
    public_vars.path = [];
    public_vars.path_index = 1;
    public_vars.indoor_prev = any(isnan(read_only_vars.gnss_position));
    public_vars.front_min_distace = 0.4;
    public_vars.init_duration = 50;
    public_vars.KF_corect_init = false;
end

% ---  Detekce prostředí (Indoor/Outdoor) ---
public_vars.indoor = any(isnan(read_only_vars.gnss_position));

%fprintf("Iterace: %i\n",read_only_vars.counter);

% --- STAVOVÝ AUTOMAT ---
switch public_vars.state
    

    case "INITIALIZING"
        % Pohyb na počátečí pozici
        public_vars.motion_vector = [0.1, 0.3]; 
        public_vars.init_counter = public_vars.init_counter + 1;
    
        % KONTROLA KOLIZE se zdí
        if read_only_vars.lidar_distances(1) < public_vars.front_min_distace
            public_vars.state = "EMERGENCY_BACK";
            public_vars.back_counter = 0; 
        end

        % Inicializace Particle Filteru 
        if public_vars.init_counter == 1 && public_vars.indoor
            public_vars = init_particle_filter(read_only_vars, public_vars);
        end
    
        % Inicializace Kalman Filteru
        if public_vars.init_counter == public_vars.init_duration
            public_vars = init_kalman_filter(read_only_vars, public_vars);
            if public_vars.indoor == 1
                public_vars.KF_corect_init = false;
            else
                public_vars.KF_corect_init = true;
            end
        end
    
        % Průběžný update filtrů (zpřesňování pozice během pohybu robota)
        if public_vars.init_counter > 1 && public_vars.indoor && public_vars.init_counter < public_vars.init_duration
            % Update PF pokud je robot uvnitř
            public_vars.particles = update_particle_filter(read_only_vars, public_vars);
        elseif public_vars.init_counter > public_vars.init_duration
            % update obou filtrů na konci inicializace
            public_vars = update_localization(read_only_vars, public_vars);
            public_vars.estimated_pose = estimate_pose(public_vars, read_only_vars);
        end
    
        % Podmínka pro ukončení inicializace (čeká ještě 10 iterací navíc)
        if public_vars.init_counter >= public_vars.init_duration + 10
            public_vars.state = "PLANNING";
            public_vars.path = [];
            public_vars.motion_vector = [0, 0];
        end


    case "EMERGENCY_BACK"
        % Ochrna před nárazem do zdi
        % otočení a popojetí
        if public_vars.back_counter < 5
            public_vars.motion_vector = [-0.6, 0.6]; 
        else
            public_vars.motion_vector = [0.5, 0.5]; 
        end
         
        public_vars.back_counter = public_vars.back_counter + 1;
        
        % Návrat do init (REINICIALIZACE)
        if public_vars.back_counter > 10
            fprintf("Zpet do init 1\n")
            public_vars.state = "INITIALIZING";
            public_vars.init_counter = 0; % Reset času pro init
        end


    case "PLANNING"
        % Pokus o naplánování cesty
        fprintf('Plánování cesty s dilatací 0,4 m\n');
        public_vars.wall_dilatation = 5;
        public_vars.path_index = 1;
        public_vars.path = plan_path(read_only_vars, public_vars);
        
        if ~isempty(public_vars.path)
            % Cesta nalezena
            public_vars.state = "NAVIGATING";
        else
            % Cesta nenalezena -> snížení dilacate stěn
            fprintf('Plánování cesty s dilatací 0,2 m\n');
            public_vars.wall_dilatation = 3;
            public_vars.path = plan_path(read_only_vars, public_vars);

            if ~isempty(public_vars.path)
                % Cesta nalezena!
                public_vars.state = "NAVIGATING";
            else
                fprintf('Nelze naplánovat cestu ani s menší dilatací! -> znovu inicializace');
                public_vars.state = "INITIALIZING"; % Restart
                fprintf("Zpet do init 3\n")
                public_vars.init_counter = 0;
            end
        end

    case "NAVIGATING"
        % Standardní jízda po cestě
        public_vars = update_localization(read_only_vars, public_vars);
        public_vars.estimated_pose = estimate_pose(public_vars, read_only_vars);
        
        public_vars = plan_motion(read_only_vars, public_vars);

        % KONTROLA KOLIZE se zdí
        if read_only_vars.lidar_distances(1) < public_vars.front_min_distace
            public_vars.state = "EMERGENCY_BACK";
            public_vars.back_counter = 0; 
            public_vars.motion_vector = [0, 0]; %zastavit pohyb
        end

        % FALEŠNÝ DOJEZD DO CÍLE -> reinit
        if public_vars.path_index == size(public_vars.path,1)
            public_vars.state = "INITIALIZING";
            fprintf("Zpet do init 2\n")
            public_vars.init_counter = 0;
            public_vars.motion_vector = [0, 0]; %zastavit pohyb
        end


        
end

end

% --- Pomocná funkce pro lokalizaci
function pv = update_localization(read_only_vars, pv)
    % Detekce přechodu
    pv.indoor = any(isnan(read_only_vars.gnss_position));
    
    if pv.indoor == 1 && pv.indoor_prev == 0
        % VJEZD DOVNITŘ: Reset částic kolem aktuálního odhadu z KF
        pv.particles = zeros(read_only_vars.max_particles/2, 3);
        pv.particles = pv.particles + pv.mu;
    end

    if pv.indoor == 0 && pv.indoor_prev == 1
        % VÝJEZD VEN: Nastavení pozice KF podle aktuálního odhadu z PF
        if pv.KF_corect_init == false
            pv.sigma = [0.1, 0.0011, 0;   
                        0.0011, 0.1, 0;   
                        0,      0,     0.1];
            pv.KF_corect_init = true;
        end
        pv.mu = pv.estimated_pose;
        pv.particles = [];
    end
    
    % update podle prostředí
    if pv.indoor
        pv.particles = update_particle_filter(read_only_vars, pv);
    else
        [pv.mu, pv.sigma] = update_kalman_filter(read_only_vars, pv);
    end
    
    pv.indoor_prev = pv.indoor;
end