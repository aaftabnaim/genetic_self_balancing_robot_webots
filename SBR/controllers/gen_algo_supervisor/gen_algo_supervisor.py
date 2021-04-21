from controller import Node,Supervisor,Keyboard,Emitter,Receiver
from population import *

superv = Supervisor()
timestep = int(superv.getBasicTimeStep())
superv.step(timestep)
sbr = superv.getFromDef("SBR")
load = superv.getFromDef("LOAD")

# The emitter to send genotype to SBR
emitter = superv.getDevice("emitter")
emitter.setChannel(1)

reciever = superv.getDevice("receiver")
reciever.enable(timestep)
reciever.setChannel(2)

#Establish Sync Between Emitter and Reciever


POPULATION_SIZE = 10
GENOTYPE_SIZE = 4
NUM_GENERATIONS = 15
bounds = [(3,17),(-3,2),(100,180),(560,640)]

# !!!!! not sure
def run_seconds(t,reset_position=False):
    
    n = 1000*t/timestep
    start = superv.getTime()
    while superv.step(timestep) != -1: 
        if superv.getTime()-start>t:
            break
        if reset_position:
            restore_robot_position()
            sbr.resetPhysics()
            load.resetPhysics()

def getPerformanceData():
    global init_translation,init_rotation,load_init_translation,load_init_rotation
    emitter.send("return_fitness".encode('utf-8'))
    while superv.step(timestep) != -1: 

        if reciever.getQueueLength()>0:
            message = reciever.getData().decode('utf-8')
            reciever.nextPacket()
            angle_fitness = float(message)

            load_translation = load.getField("translation").getSFVec3f()
            load_rotation = load.getField("rotation").getSFRotation()
            load_t_cost = sum([(i1-i2)**2 for i1,i2 in zip(load_translation,load_init_translation)])
            load_r_cost = sum([(i1-i2)**2 for i1,i2 in zip(load_rotation,load_init_rotation)])
            

            sbr_translation = sbr.getField("translation").getSFVec3f()
            sbr_rotation = sbr.getField("rotation").getSFRotation()
            sbr_t_cost = sum([(i1-i2)**2 for i1,i2 in zip(sbr_translation,init_translation)])
            sbr_r_cost = sum([(i1-i2)**2 for i1,i2 in zip(sbr_rotation,init_rotation)])
            #print("Angle Fitness - ",angle_fitness)
            #print("Load Fitness - ",(load_r_cost+load_t_cost))
            #print("Robot T Fitness ",(sbr_r_cost+sbr_t_cost))
            return angle_fitness+((load_r_cost+load_t_cost)*100+(sbr_r_cost+sbr_t_cost))*30
            

            
    
def send_genotype(genotype):
    genotype_string = [str(g) for g in genotype]
    genotype_string = ','.join(genotype_string)
    
    emitter.send(genotype_string.encode('utf-8'))
    
def restore_robot_position():
    global init_translation,init_rotation
    sbr_translation.setSFVec3f(init_translation)
    sbr_rotation.setSFRotation(init_rotation)
    load_translation.setSFVec3f(load_init_translation)
    load_rotation.setSFRotation(load_init_rotation)
    

def evaluate_genotype(genotype):
    #test_genotype = [6.70891752445785, -2.984975676757869, 148.50048150101875, 655.0303108723926]
    # send genotype to robot
    send_genotype(genotype)
    
    # run for some time
    run_seconds(90)
    #store fitness
    fitness = getPerformanceData()
    #print("Supervisor:Fitness of ",genotype," - %f "%(fitness))
   
    sbr.resetPhysics()
    restore_robot_position()
    
    run_seconds(5,True)
    
    sbr.resetPhysics()
    restore_robot_position()
    
    # reset physics
    return fitness

    

def run_optimization():
    global population
    
    print("---\n")
    print("Starting Optimization")
    print("Population Size %i , Genome Size %i"%(POPULATION_SIZE,GENOTYPE_SIZE))

    for gen in range(NUM_GENERATIONS):
        population_fitness = []
        for ind in range(POPULATION_SIZE):
            print("Generation %i , Genotype %i "%(gen,ind))

            #population_get_genotype
            genotype = population[ind]

            #evaluate_genotype
            fitness = abs(evaluate_genotype(genotype))
            
            population_fitness.append(fitness)

        best_fitness,best_fitness_val = population_get_fittest(population,population_fitness)
        average_fitness = population_get_average_fitness(population_fitness)
        print("Best Fitness ",best_fitness)
        print("Best Fitness Value - %f"%best_fitness_val)
        print("Average Fitness - %f"%average_fitness)

        if(gen < NUM_GENERATIONS-1):
            population = population_reproduce(population,population_fitness)

    return best_fitness

     
def main():
    #Initiate keyboard 
    global init_translation,init_rotation,population,sbr_translation,sbr_rotation,load_init_translation,load_init_rotation,load_translation,load_rotation
    
    keyb = Keyboard()
    keyb.enable(timestep)
    count = 0

    sbr_translation = sbr.getField("translation")
    sbr_rotation = sbr.getField("rotation")
    init_translation = (sbr_translation.getSFVec3f())
    init_rotation = (sbr_rotation.getSFRotation())
    
    load_translation = load.getField("translation")
    load_rotation = load.getField("rotation")
    load_init_translation = (load_translation.getSFVec3f())
    load_init_rotation = (load_rotation.getSFRotation())   
    

    population = population_create(POPULATION_SIZE,GENOTYPE_SIZE,bounds)

    fittest = run_optimization()
    
    send_genotype(fittest)

    #restore robots position
    restore_robot_position() 
    
  
    while superv.step(timestep) != -1:
        key = keyb.getKey()
        
        if key==ord('Q'):
            quit()   
   

main()
# Enter here exit cleanup code.
