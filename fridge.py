import PyDSTool as de
import matplotlib.pyplot as plt

# control this with the following
t_kitchen = 20.0;         # kitchen (ambient temperature)
fridge_cap = 100000/2;       # thermal capacity, Joule/deg
freeze_cap = 100000/2;       # thermal capacity, Joule/deg

# initial state of the fridge, kitchen temperature
icdict = { 't1': t_kitchen, 't2' : t_kitchen }

# parameter dictionary
pardict = dict(Ck1 = fridge_cap, Ck2 = freeze_cap, R1 = 10, R2 = 4, 
               P1 = 400, P2 = 400, tenv = t_kitchen)

# differential equation for fridge with motor on
# since the fridge is a fairly slow system, the step size can be
# larger, 10 s in this case
fridge_on_args = {
    'pars' : pardict,
    'varspecs' : { 't1': '1/Ck1*(R1*(tenv - t1) - P1)',
                   't2': '1/Ck2*(R2*(tenv - t2) - P2)', }, 
    'name' : 'fridge_on',
    'algparams' : { 'init_step' : 10 },
    'tdata' : [0, 24*3600] }

# The generators are the differential equation solvers
# the 'embed' creates a re-usable model for these
all_model_names = ['fridge_working']
fridge_working = de.Generator.Vode_ODEsystem(fridge_on_args)

# calculate the response, 4 hours
traj = fridge_working.compute(trajname='24hours', ics=icdict)

# and plot
plotData = traj.sample(dt = 60)


final_temp = plotData['t1'][-1]
initial_temp = plotData['t1'][0]
print(final_temp)

for j in range(len(plotData['t1'])):
    if plotData['t1'][j] <= t_kitchen - 0.9*(initial_temp-final_temp):
        t90 = plotData['t'][j]
        break
print(t90)

for i in range(len(plotData['t1'])):
    if plotData['t1'][i] <= t_kitchen - 0.63*(initial_temp-final_temp):
        t63 = plotData['t'][i]
        break
print(t63)



plt.ylabel('t1, t2 [C]')
plt.xlabel('time [s]')
friline = plt.plot(plotData['t'], plotData['t1'],label='fridge')
freline = plt.plot(plotData['t'], plotData['t2'],label='freezer')
plt.legend()
plt.show()
