
    if turningLeft: 
        print("turning left" + str(straight))
        if gsr[1] < GROUND_SENSOR_THRESHOLD and gsr[0] > GROUND_SENSOR_THRESHOLD and gsr[2] > GROUND_SENSOR_THRESHOLD: 
            turningLeft = False
        else : 
            while robot.step(SIM_TIMESTEP) != -1:
                if straight: 
                    vL = -1*MAX_SPEED
                    vR = MAX_SPEED
                    straight = False
                else: 
                    vL = MAX_SPEED
                    vR = MAX_SPEED
                    straight = True
                if gsr[1] < GROUND_SENSOR_THRESHOLD and gsr[0] > GROUND_SENSOR_THRESHOLD and gsr[2] > GROUND_SENSOR_THRESHOLD: 
                    turningLeft = False
    elif turningRight: 
        print("turning right" + str(turningRight))
        if straight > 2: 
            straight = 0
            vL = MAX_SPEED
            vR = -1*MAX_SPEED
        else: 
            straight += 1
            vL = MAX_SPEED
            vR = MAX_SPEED
    else: 