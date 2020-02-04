from ue.model import TrafficFlowModel

graph = [
    ('1', ['2']),
    ('2', ['1', '9', '3', '6']),
    ('3', ['2', '4', '11', '7']),
    ('4', ['3']),
    ('5', ['6']),
    ('6', ['5', '2', '10', '7']),
    ('7', ['6', '3', '8', '12']),
    ('8', ['7']),
    ('9', ['2']),
    ('10', ['6']),
    ('11', ['3']),
    ('12', ['7'])
]

free_time = [
    0.58,
    0.58, 0.33, 0.46, 0.55,
    0.46, 0.54, 0.33, 0.56,
    0.54,
    0.58,
    0.58, 0.55, 0.54, 0.47,
    0.47, 0.56, 0.60, 0.57,
    0.60,
    0.33,
    0.54,
    0.33,
    0.57
]

capacity = [1800] * len(free_time)

origins = [str(i) for i in range(1, 13)]
destinations = [str(i) for i in range(1, 13)]

demand = []
for i in range(12):
    for j in range(12):
        if i == j:
            demand.append(0)
        else:
            demand.append(60)

# Initialize the model by data
mod = TrafficFlowModel(graph, origins, destinations, demand, free_time, capacity)
# Change the accuracy of solution if necessary
mod._conv_accuracy = 1e-6
# Set the precision of display, which influences only the digit of numerical component in arrays
mod.set_disp_precision(4)
# Solve the model by Frank-Wolfe Algorithm
mod.solve()

mod.report()
mod._formatted_solution()