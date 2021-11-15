#!/usr/bin/python

import random


def generate_numbers(n, lower_boundary, upper_boundary, decimal_precision):
    random_nums = []
    for i in range(n):
        random_nums.append(round(random.uniform(lower_boundary, upper_boundary), decimal_precision))
    print(len(random_nums))
    return random_nums


def write_to_file(random_nums, filename):
    with open(filename, "w") as outfile:
        outfile.write(",".join(str(item) for item in random_nums))


if __name__ == "__main__":
    write_to_file(generate_numbers(1000, 1.01, 99.99, 2), "random_nums_A.txt")
    write_to_file(generate_numbers(1000, 1.01, 99.99, 2), "random_nums_B.txt")
# print("Random float number with 2 decimal places")
# num2 = round(random.random(), 2)
# print(num2)
#
# num2 = round(random.uniform(0.00, 99.99), 2)
# print(num2)
#
# print("Random float number with 1 decimal places")
# print(round(random.random(), 1))
# print(round(random.uniform(0.00, 99.99), 2))