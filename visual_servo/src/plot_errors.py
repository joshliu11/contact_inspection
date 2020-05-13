import matplotlib.pyplot as plt

def parse(filepath):

    # parse the file
    errors = [line.rstrip('\n') for line in open(filepath)]

    return errors

if __name__ == '__main__':

    # obtain error vector
    errors = parse("datafiles/errors.txt")
    errors = [float(i) for i in errors]

    # plot result
    plt.plot(errors)
    plt.xlabel('frames')
    plt.ylabel('percent difference')
    plt.show()
