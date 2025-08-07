class PyTask:
    func = None
    repair_func = None
    def __init__(self, name, func, repair_func):
        self.name = name
        self.func = func
        self.repair_func = repair_func
    
    def run(self, *data):
        if self.func:
            self.func(data[0])
    
    def repair_run(self, *data):
        if self.repair_func:
           self.repair_func(data[0])

class PyMission:
    name = ""
    task_list = []
    def __init__(self, name, task_list):
        self.name = name
        self.task_list = task_list
