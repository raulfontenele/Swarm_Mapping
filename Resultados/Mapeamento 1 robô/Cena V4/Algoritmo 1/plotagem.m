    hold on;

%Paredes em volta
extY = [-2.5 2.5 2.5 -2.5 -2.5];
extX = [-2.5 -2.5 2.5 2.5 -2.5];

plot(extY,extX,'r', 'LineWidth',4)

%Paredes internas
p1x = [-2.5	-2.5	1	1	-2.5];
p1y = [0.65	0.5	0.5	0.65 0.65];

p2x = [-0.35	-0.35	-0.5	-0.5	-0.35];
p2y = [-0.5	-2.5	-2.5	-0.5	-0.5];

plot(p1y,p1x,p2y,p2x,'r', 'LineWidth',1)


for i = 1:length(arestas)
    plot([arestas(i,2),arestas(i,5)],[arestas(i,1),arestas(i,4)])
end
scatter(coord(:,2),coord(:,1))
hold off
