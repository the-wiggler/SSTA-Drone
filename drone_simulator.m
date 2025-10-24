figure;
hold on;
grid on;
view(3);
axis equal;
xlim([-10 10]); ylim([-10 10]); zlim([-10 10]);
xlabel('X'); ylabel('Y'); zlabel('Z');

t = linspace(0, 4*pi, 100);
x_pos = 3*cos(t);
y_pos = 3*sin(t);
z_pos = 0.3*t;

h_obj = plot3(x_pos(1), y_pos(1), z_pos(1), 'ro', 'MarkerSize', 20, 'MarkerFaceColor', 'r');


for i = 1:length(t)
    set(h_obj, 'XData', x_pos(i), 'YData', y_pos(i), 'ZData', z_pos(i));
    
    drawnow;
    pause(0.03);
end
