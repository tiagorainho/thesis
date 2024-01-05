
from random import random
from math import exp
import matplotlib.pyplot as plt
import numpy as np

P_initial = 0.75
P_min = 0.05
P_acceleration = 0.5

num_tests = 10000000
num_checks = 500

iteration = 2



def check_area(x):
    return random() < ((P_initial - P_min)/exp(P_acceleration*x))+P_min

if __name__ == '__main__':

    num_successful_tests = [0 for _ in range(num_checks)]


    for i in range(num_tests):
        counter = 0
        for check_i in range(num_checks):
            if check_area(2):
                counter += 1
        num_successful_tests[counter] += 1


    # x -> numero de testes q correram bem
    # y -> numbero de vezes q aconteceu


    plt.plot([i for i in range(num_checks)], num_successful_tests)
    plt.xlabel('Points of Interest checked')
    plt.ylabel('Number of tests')

    # plt.yticks(np.arange(0, num_tests, num_tests/10*5))
    # plt.title('Assumed already 2 iterations')
    plt.show()