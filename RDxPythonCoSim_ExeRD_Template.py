from fmpy import read_model_description, extract
from fmpy.fmi2 import FMU2Slave
import numpy as np
import shutil
import matplotlib.pyplot as plt
import sys
print("\n" * 100)

##### Begin - User Statement
##### define the model name and simulation parameters
# define the fmu file name
fmu_filename = ''

def userFunction(time, var_in, var_out, inputs, outputs, step_size):
    # in this function,
    # input means (G)PlantInput in RecurDyn
    # output means (G)PlantOutPut
    # The variable name input is declared in terms of RecurDyn.
    # In other words, the variable input is the result of this function.

    # Calculate inputs to RecurDyn. At the first call outputs are in the initial state

    # $$$$$$$$$$$$$$$$$$$   Add your code here



    # $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    # the input is given to RecurDyn and the reaction is returned from RecurDyn.
    fmu.setReal(var_in, inputs)
    fmu.doStep(currentCommunicationPoint=time, communicationStepSize=step_size)
    outputs = fmu.getReal(var_out)
    return inputs, outputs
##### End - User Statement

def DrawPlotStates(states, inputNum, outputNum):
    statesTrans = np.array(states).T
    timeResult = statesTrans[0]
    inputIndex = 1
    outputIndex = 1 + inputNum
    plotIndex = 1
    inputCount = 1
    outputCount = 1

    if inputNum == 1 and outputNum == 1:
        plt.subplot(2, 1, 1)
        plt.plot(timeResult, statesTrans[1])
        plt.title('GPlantInput1')
        plt.subplot(2, 1, 2)
        plt.plot(timeResult, statesTrans[2])
        plt.title('GPlantInput2')
    else:
        for i in range(inputIndex, inputIndex + inputNum):
            plt.subplot(inputNum, outputNum, plotIndex)
            plt.plot(timeResult, statesTrans[i])
            plt.title('GPlantInput' + str(inputCount))
            plotIndex += 1
            inputCount += 1

        for i in range(outputIndex, outputIndex + outputNum):
            plt.subplot(inputNum, outputNum, plotIndex)
            plt.plot(timeResult, statesTrans[i])
            plt.title('GPlantOutput' + str(outputCount))
            plotIndex += 1
            outputCount += 1

    plt.tight_layout()
    plt.show()

# read the model description
model_description = read_model_description(fmu_filename)

# collect the value references
var_input = {}
var_output = {}
start_time = 0.0
stop_time = 0.0
step_size = 0.0
ninput = 0
noutput = 0

for variable in model_description.modelVariables:
    if "RD_StopTimeDefined" in variable.name:
        stop_time = float(variable.start)
        stop_time = stop_time
    if "InterfaceStepSize" in variable.name:
        step_size = float(variable.start)
    if "nPinput" in variable.name:
        ninput = int(variable.start)
    if "nPoutput" in variable.name:
        noutput = int(variable.start)
    if "Input Data1 of Plant(RD)" in variable.description:
        var_input[variable.name] = variable.valueReference
    if "Output Data1 of Plant(RD)" in variable.description:
        var_output[variable.name] = variable.valueReference

# IO variable reference to list
var_in = list(var_input.values())
var_out = list(var_output.values())

# extract the FMU 
unzipdir = extract(fmu_filename)

fmu = FMU2Slave(guid=model_description.guid,
                unzipDirectory=unzipdir,
                modelIdentifier=model_description.coSimulation.modelIdentifier,
                instanceName='instance1')

# initialize
fmu.instantiate()
fmu.setupExperiment(startTime=start_time)
fmu.enterInitializationMode()
fmu.exitInitializationMode()

# simulation standby
time = start_time
states = [[0 for _ in range(len(var_in) + len(var_out) + 1)]]  # time,PIN,POUT 총 갯수를 배열로
# states = np.zeros(len(var_in) + len(var_out))
outputs = [0 for _ in range(len(var_out))]

# simulation loop
while time <= (stop_time + 1):
    inputs = np.zeros(len(var_in)).tolist()
    inputs, outputs = userFunction(time, var_in, var_out, inputs, outputs, step_size)
    states.append(([time] + inputs + outputs))

    if time > stop_time:
        break
    time += step_size

# Simulation Terminate
fmu.terminate()
fmu.freeInstance()

# clean up
shutil.rmtree(unzipdir, ignore_errors=True)

DrawPlotStates(states, len(var_in), len(var_out))

