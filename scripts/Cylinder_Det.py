import numpy as np

res = 60   # Resolução do lidar
r = 25     #[cm] - Raio dos cilindros (Especificado no edital)
d_min = 10 #[cm] - Distancia minima permitida pro Drone chegar perto de algo

sensor = np.array([10,12,10,0,0,0,0,0,0,0,30,30,34,37,32,36,38,30,30,30,30,30,30,0,0,12,14,17,40,45,50,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 10, 11, 13, 11]) # Dado do lidar com tamanho "res"

def prop_max ():
    return (1/np.pi)*np.arccos((r+d_min)/(np.sqrt(2*r**2+2*r*d_min+d_min**2)))

def varMax (d_min):
    return np.sqrt(2*r**2+2*r*d_min+d_min**2) - d_min

def varMax_teorico ():
    return np.sqrt(2)*r - r

prop_max = prop_max()
varMax = varMax_teorico()

def detectar ():

    objetos = []                    # A lista de objetos que vão ser devolvidas
    #sensor_ext = sensor.append(sensor[0:int(prop_max*res)])    # Aqui eu tava tentando extender a lista, mas é mais complicado que fazer isso
    dists = sensor[sensor>0]        # Lista ignorando os zeros
    indices = [ i for i in range(res) if sensor[i] != 0 ]
    #dists = np.append(dists, -1000) # Indica o final dessa lista pro código funfar certinho
    temp = []                       # Lista temporaria pra guardar os objetos que vão ser inseridos em "objetos"
    obj_indices = []                    # Lista dos indices da lista "dists" que representam os cilindros
    indices_temp = []               # Armazena os indices que vao ser colocados em "indices"

    print(indices[0])

    obj = 0
    temp.append( dists[0] )
    indices_temp.append( indices[0] )
    i = 1
    while i < len(dists):

        if (abs(dists[i] - dists[i-1]) <= varMax): # Vê se o próximo elemento da lista tem uma variação "permitida" de acordo com a fórmula
            temp.append(dists[i])

        elif ((len(temp)/res) > prop_max):      # Se o proximo elemento não for mais um objeto de acordo com varMax, o codigo analisa o tamanho do objeto e faz a proporção pra ver se pode ser um cilindro (se nao, é chao)
            temp = []                           # Nesse caso, seria chão, então reinicializa as listas temporarias
            indices_temp = []
            temp.append(dists[i])
            indices_temp.append( indices[i] )

        else:                                   # Se nenhum dos de cima, então é um cilindro. Ai o codigo bota nas listas "objetos" e "indices" e dps reinicializa os temporarios
            objetos.append(temp)
            indices_temp.append( indices[i-1] )
            obj_indices.append( indices_temp )

            temp = []
            indices_temp = []
            temp.append(dists[i])
            indices_temp.append( indices[i] )

        i += 1

    if sensor[-1] != 0 and indices[0] == 0:
        
        while abs(dists[i% len(dists) ] - dists[ (i-1) % len(dists) ]) <= varMax:
            temp.append(dists[i% len(dists) ])
            i += 1
        if len(temp)/res < prop_max:
            objetos.append(temp)
            indices_temp.append( indices[(i-1) % len(dists)] )
            obj_indices.append(indices_temp)

        del objetos[0]
        del obj_indices[0]

    elif len(temp) > 0:
        objetos.append(temp)
        indices_temp.append( indices[i-1] )
        obj_indices.append(indices_temp)
                

    return objetos, obj_indices


objetos, indices = detectar()

print("As distancias dos objetos detectados são: {}".format(objetos))
print("Os intervalos dos indices dos objetos detectados são: {}".format(indices))
print(prop_max*res)
