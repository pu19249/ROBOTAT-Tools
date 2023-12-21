import csv
'''
This file was used to test how to write to a CSV file and add values as they were received. In this case a continuous incremental counter representing (x, y, theta).
'''
x = [0]
y = [0]
theta = [0]

file_name = input('Type file name for the csv: ')
with open(file_name+'.csv', 'a', newline='') as file:
    writer = csv.writer(file)
    field = ["x position", "y position", "orientation"] # titles of the columns

    # Write the field names only once, not in every iteration
    writer.writerow(field)

    while True:
        x[0] = x[0] + 1
        y[0] = y[0] + 1
        theta[0] = theta[0] + 1
        writer.writerow([x[0], y[0], theta[0]])