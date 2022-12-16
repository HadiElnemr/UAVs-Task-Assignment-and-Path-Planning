from matplotlib import pyplot as plt
  
def plot_fitnesses(n_iter, best_fitnesses):
    fig1, ax1 = plt.subplots()
    ax1.set_xlabel('Iteration no.')
    ax1.set_ylabel('fitness value')

    ax1.plot([i for i in range(1,n_iter+1)], best_fitnesses)
    plt.show()