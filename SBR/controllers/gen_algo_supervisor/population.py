import pandas as pd
import random

ELITE_PART = 0.4
MUTATION_PROBABILITY = 0.45
MUTATION_DEVIATION = 0.4

def population_create(p_size,geno_size,bounds):
    
    p = [ random_param(geno_size,bounds) for i in range(p_size)]
    return p
    
def random_param(g,b):
    return [random.uniform(b[i][0],b[i][1]) for i in range(g)]

def population_reproduce(p,fitness):

    size_p = len(p)
    new_p = []
    
    dataframe = pd.DataFrame({"Param":p,"Fitness":fitness})
    dataframe = dataframe.sort_values(['Fitness'])
    dataframe = dataframe.reset_index(drop=True)    

    sorted_p = dataframe['Param'].tolist()

    elite_part = round(ELITE_PART*size_p)
    new_p = new_p + sorted_p[:elite_part]

    for i in range(size_p-elite_part):
        mom = p[random.randint(0,size_p-1)]
        dad = p[random.randint(0,size_p-1)]
        child = crossover(mom,dad)
        child = mutate(child)
        new_p.append(child)
    
    return new_p

def population_get_fittest(p,f):
    dataframe = pd.DataFrame({"Param":p,"Fitness":f})
    dataframe = dataframe.sort_values(['Fitness'])
    dataframe = dataframe.reset_index(drop=True)
    
    return dataframe[0:1]['Param'].values[0],dataframe[0:1]['Fitness'].values[0]

def population_get_average_fitness(f):
    return sum(f)/len(f)

def crossover(p1,p2):
    
    crossover = []
    locii = [random.randint(0,8) for _ in range(len(p1))]
    
    for i in range(len(p1)):
        if locii[i]>4:
            crossover.append(p2[i])
        else:
            crossover.append(p1[i])
        
    return crossover

def mutate(c):
    size = len(c)
    for i in range(size):
        if random.random()< MUTATION_PROBABILITY:
            if i==0:
                c[1] += random.gauss(0,2)
            elif i==1:
                c[2] += random.gauss(0,1.5)
            else:
                c[i] += random.gauss(0,3)*10
            
    return c   
    

    
    

