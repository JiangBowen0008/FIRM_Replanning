import numpy as np

import seaborn as sns
import matplotlib.pyplot as plt

sns.set_style('whitegrid')

def main():
    plt.figure()

    labels = ['G1', 'G2', 'G3', 'G4', 'G5']
    t_local = [20, 34, 30, 35, 27]
    t_global = [25, 32, 34, 20, 25]

    x = np.arange(len(labels))  # the label locations
    width = 0.35  # the width of the bars

    fig, ax = plt.subplots()
    rects1 = ax.bar(x - width/2, men_means, width, label='Men')
    rects2 = ax.bar(x + width/2, women_means, width, label='Women')

    # Add some text for labels, title and custom x-axis tick labels, etc.
    ax.set_ylabel('Scores')
    ax.set_title('Scores by group and gender')
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.legend()

    ax.bar_label(rects1, padding=3)
    ax.bar_label(rects2, padding=3)

    fig.tight_layout()

    plt.show()




if __name__ == '__main__':
    main()