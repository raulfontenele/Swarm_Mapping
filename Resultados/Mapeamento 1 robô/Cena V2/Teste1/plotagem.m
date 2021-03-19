hold on;

%Paredes em volta
extY = [-2.5 2.5 2.5 -2.5 -2.5];
extX = [-2.5 -2.5 2.5 2.5 -2.5];

plot(extX,extY,'r', 'LineWidth',4)

% %Paredes internas
% blcX1 = [1.5 1.5 0.5 0.5 1.5];
% blcY1 = [-0.5 -1.5 -1.5	-0.5 -0.5];
% 
% blcX2 = [-0.5 -0.5 -1.5 -1.5 -0.5];
% blcY2 = [1.5 0.5 0.5 1.5 1.5];
% 
% plot(blcX1,blcY1,blcX2,blcY2,'r', 'LineWidth',1)


for i = 1:length(arestas)
    plot([arestas(i,1),arestas(i,4)],[arestas(i,2),arestas(i,5)])
end

scatter(coord(:,1),coord(:,2))
hold off
