import json

def analysis():
    file = open('stepMap.txt','r')
    lines = file.readlines()

    radius = 0.5

    #ids  = [1,2,3]
    coordinates = []
    coordUnique = []
    for line in lines:
        line = line.replace("'",'"')
        struct =json.loads(line)
        
        #coordinates.append(struct["coord"])
        flag = False
        for node in coordUnique:
            if (  (struct["coord"][0] - node["coord"][0])**2 + (struct["coord"][1] - node["coord"][1])**2   )**(1/2) < radius:
                flag =  True
                index = coordUnique.index(node)
                coordUnique[index]["number"]+=1
                break
        if flag == False:
            coordUnique.append({
                "coord" : struct["coord"],
                "number" : 1
            })
        coordinates.append(struct["coord"])
        listSorted = sorted(coordUnique, key=lambda x: x["number"])
        

    with open('nodes.txt', 'w', encoding='utf-8') as file:
        for node in listSorted:
            file.write(str(node) + '\n')
    print(coordUnique)


    '''
    coordSeries = pd.Series(coordinates)
    unique = coordSeries.unique()
    print(unique)
    '''


analysis()