%% Matriz de distâncias
len = length(coord);
matrix = zeros(len);

for i = 1:len
    for j = 1:len
        matrix(i,j) = sqrt(sum(abs(coord(i,:) - coord(j,:)).^2));
    end
end

matrix