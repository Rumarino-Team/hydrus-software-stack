import pymission

def hello(data):
    print("Hello from Python!")

def new():
    task = pymission.PyTask("pytask-example", hello, None)
    task_list = [task]
    mission = pymission.PyMission("pymission-example", task_list)
    return mission