%% Entrega 3 

%% Que metodo de integracion
fprintf('Métodos de integración:\n')
fprintf('  1.Euler\n')
fprintf('  2. Runge-Kutta 2do orden\n')
fprintf('  3. Runge-Kutta 4do orden\n')
metodo = input('Elige un método: ');

%% Definimos parametros
numero_cables = 16;
radio_toroide = 0.1;
porcentaje_dibujar = 0.8;
radio_dibujar = radio_toroide * porcentaje_dibujar;
porcentaje_r_limite = porcentaje_dibujar-0.05;  
radio_limite = radio_toroide * porcentaje_r_limite; 

corriente_estatica =  5000;
corriente_rescate = 50000;

q = 1.6e-19;         
m = 1.67e-27;  
miu0= 4*pi*1e-7;

% posicion y velocidad incial
r0= [0.01, 0.01];
dir=rand*2*pi;
vi=1e3;
v0 = [vi*cos(dir), vi*sin(dir)];



%% posiciones de los cables
angulos_cables = linspace(0,2*pi,numero_cables+1);
angulos_cables(end) = [];

% calculamos posiciones
x_cables = radio_toroide * cos(angulos_cables);
y_cables = radio_toroide * sin(angulos_cables);

%% simulacion
dt = 1e-9;
pasos = 500000;

radio = zeros(pasos+1, 2);
v = zeros(pasos+1,2);

radio(1,:)=r0;
v(1,:)=v0;

%Figura
figure('Color','k')
hold on
grid on
axis equal

xlim([-0.12 0.12])
ylim([-0.12 0.12])


%limites de cojfinamiento
theta_circulo = linspace(0,2*pi,200);

x_limite = radio_limite *cos(theta_circulo);
y_limite = radio_limite * sin(theta_circulo);

plot(x_limite, y_limite, 'r--', 'LineWidth', 1.5);


x_limite_real = radio_dibujar *cos(theta_circulo);
y_limite_real = radio_dibujar * sin(theta_circulo);

plot(x_limite_real, y_limite_real, 'r-', 'LineWidth', 1.5);

x_exterior = radio_toroide * cos(theta_circulo);
y_exterior = radio_toroide * sin(theta_circulo);
plot(x_exterior, y_exterior, 'w-', 'LineWidth', 1.5);

%cables
hCables =zeros(1, numero_cables);

for i=1:numero_cables
    hCables(i)=plot(x_cables(i),y_cables(i),'wo','MarkerFaceColor', [0.3 0.3 0.3], ...
        'MarkerSize', 10);
end

I = ones(1, numero_cables) * corriente_estatica;

%% Loop principal para calcular la posicion de la particula
%try para evitar un error de cerrar
try
    for n = 1:pasos
        % Actualizamos velocidad y posicion
    
        %distancia del centro
        distancia_centro = norm(radio(n,:));
    
        % se reinician corrientes
        % (considerando que el la ronda pasada se adaptaron)
        I(:)= corriente_estatica;
    
        % Parte de control activo
        if distancia_centro>radio_limite
            % calcular la direccion de escape
            angulo = atan2(radio(n,2),radio(n,1));
            
            if angulo<0
                angulo = angulo + 2*pi;
       
            end
    
            %encontrar el cable mas cercano
            % encuentra cabke omas cerca anugularmente
            [~, cable_mas_cercano] = min(abs(angulos_cables - angulo));
            
            %Cables cercanos
            cable_mas_cercano_izq =mod(cable_mas_cercano -2, numero_cables) + 1;
            cable_mas_cercano_der =mod(cable_mas_cercano, numero_cables) + 1;
    
            % aplicamos corrientes de rescate
    
            I(cable_mas_cercano) = corriente_rescate; 
            I(cable_mas_cercano_izq) = corriente_rescate*0.5; 
            I(cable_mas_cercano_der) = corriente_rescate*0.5; 
    
            % hacemos que se vea
            set(hCables,'MarkerFaceColor', [0.3 0.3 0.3])
            set(hCables(cable_mas_cercano), 'MarkerFaceColor', 'r');
            set(hCables([cable_mas_cercano_izq cable_mas_cercano_der]), 'MarkerFaceColor', [1 0.5 0]);
    
        % si sigue adentro
        else
            %mantenemos todo normal
            set(hCables, 'MarkerFaceColor', [0.3 0.3 0.3]);
    
        end
    
        %Campo magnetico
        B_z=0;
        
    
        for i=1:numero_cables
            dx = radio(n,1) - x_cables(i);
            dy = radio(n,2) - y_cables(i);
    
            distancia= sqrt(dx^2 + dy^2);
    
            if distancia<1e-5
                distancia=1e-5;
            end
    
            % Alternamos direccion de corriente
            % pares adyacentes puntan en direcciones opuestas  (confinamiento)
            if mod(i,2)==0
                corriente= I(i);
            else
                corriente = -I(i);
            end
    
            % Calcular el campo magnético en z
            B_z = B_z + (miu0 * corriente) / (2 * pi * distancia);
        end
    
        % Fuerza de lorentz
        a_x=(q*v(n,2)*B_z)/ m;
        a_y = -(q*v(n,1)*B_z)/ m;
    
        a=[a_x a_y];
    
        %% AQUI EMPIEZAN LO DE INTEGRACION 
        switch metodo
    
        %% Integracion Euler
            case 1
                radio(n+1,:) = radio(n,:) + v(n,:)*dt;
                v(n+1,:) = v(n,:) + a*dt;
    
        %% Integracion Runge-Kutta
        %estado actual k1
        
            case 2
                k1_r = v(n,:);
                k1_v = a; 
            
                %k2
                r_pred = radio(n,:) + k1_r * dt;
                v_pred = v(n,:)    + k1_v * dt;
            
                Bz_pred = 0;
                for i = 1:numero_cables
                    dx = r_pred(1) - x_cables(i);
                    dy = r_pred(2) - y_cables(i);
                    distancia = sqrt(dx^2 + dy^2);
                    if distancia < 1e-5
                        distancia = 1e-5;
                    end
                    if mod(i,2) == 0
                        corriente = I(i);
                    else
                        corriente = -I(i);
                    end
                    Bz_pred = Bz_pred + (miu0 * corriente) / (2*pi*distancia);
                end
            
                k2_r = v_pred;
                k2_v = [(q * v_pred(2) * Bz_pred) / m, -(q * v_pred(1) * Bz_pred) / m];
            
                radio(n+1,:) = radio(n,:) + 0.5*(k1_r + k2_r)*dt;
                v(n+1,:)     = v(n,:)     + 0.5*(k1_v + k2_v)*dt;
                
    
        %% Integracion Runhe-Kutta Cuarto Orden
            case 3
            k1_r = v(n,:);
            k1_v = a;
        
            r2 = radio(n,:) + k1_r * (dt/2);
            v2 = v(n,:)+ k1_v * (dt/2);
            
            Bz2 = 0;
        
            for i = 1:numero_cables
                dx = r2(1) - x_cables(i);
                dy = r2(2) - y_cables(i);
                distancia = sqrt(dx^2 + dy^2);
                if distancia < 1e-5; distancia = 1e-5; end
                if mod(i,2) == 0; corriente = I(i);
                else; corriente = -I(i); end
                Bz2 = Bz2 + (miu0 * corriente) / (2*pi*distancia);
            end
            k2_r = v2;
            k2_v = [(q * v2(2) * Bz2) / m, -(q * v2(1) * Bz2) / m];
        
            
            r3 = radio(n,:) + k2_r * (dt/2);
            v3 = v(n,:)     + k2_v * (dt/2);
            
            Bz3 = 0;
            for i = 1:numero_cables
                dx = r3(1) - x_cables(i);
                dy = r3(2) - y_cables(i);
                distancia = sqrt(dx^2 + dy^2);
                if distancia < 1e-5; distancia = 1e-5; end
                if mod(i,2) == 0; corriente = I(i);
                else; corriente = -I(i); end
                Bz3 = Bz3 + (miu0 * corriente) / (2*pi*distancia);
            end
            k3_r = v3;
            k3_v = [(q * v3(2) * Bz3) / m, -(q * v3(1) * Bz3) / m];
        
            r4 = radio(n,:) + k3_r * dt;
            v4 = v(n,:)     + k3_v * dt;
            
            Bz4 = 0;
            for i = 1:numero_cables
                dx = r4(1) - x_cables(i);
                dy = r4(2) - y_cables(i);
                distancia = sqrt(dx^2 + dy^2);
                if distancia < 1e-5; distancia = 1e-5; end
                if mod(i,2) == 0; corriente = I(i);
                else; corriente = -I(i); end
                Bz4 = Bz4 + (miu0 * corriente) / (2*pi*distancia);
            end
            k4_r = v4;
            k4_v = [(q * v4(2) * Bz4) / m, -(q * v4(1) * Bz4) / m];
        
            radio(n+1,:) = radio(n,:) + (dt/6) * (k1_r + 2*k2_r + 2*k3_r + k4_r);
            v(n+1,:) = v(n,:) + (dt/6) * (k1_v + 2*k2_v + 2*k3_v + k4_v);
        end
        %% ANIMACION
        if mod(n, 100)==0    
            plot(radio(1:n,1),radio(1:n,2),'b')
            drawnow limitrate 
         
        end
    end
    
    title('Confinamiento magnetico activo')
    xlabel('x (m)')
    ylabel('y (m)')
catch
end
