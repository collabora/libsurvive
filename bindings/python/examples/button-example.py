import sys
import pysurvive

ctx = pysurvive.init(sys.argv)

if ctx is None: # implies -help or similiar
    exit(-1)

def button_func(obj, eventtype, buttonid, axisids, axisvals):
    if eventtype == pysurvive.SURVIVE_INPUT_EVENT_BUTTON_DOWN:
        eventstring = "DOWN"
    elif eventtype == pysurvive.SURVIVE_INPUT_EVENT_BUTTON_UP:
        eventstring = "UP"
    else:
        eventstring = "%d" % (eventtype)
    print("Button %d on %s generated event %s"%(buttonid, obj.contents.codename.decode('utf8'), eventstring))


keepRunning = True

pysurvive.install_button_fn(ctx, button_func)

while keepRunning and pysurvive.poll(ctx) == 0:
    pass

pysurvive.close(ctx)
