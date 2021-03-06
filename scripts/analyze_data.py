import matplotlib.pyplot as plt

from genetic_algorithm import GeneticAlgorithm

generation_count = 10

xs = range(generation_count)

ys = []
ga = GeneticAlgorithm(0, load=False)
for i in xs:
    ga.load(i)
    ys.append(ga.avg_capture_rate())

# make a plot of average capture rate for generations
plt.plot(xs, ys, color='red', linestyle='-')

plt.title("Average capture rate of each generation")
plt.xlabel("Generation #")
plt.ylabel("Capture Rate")
plt.show()
