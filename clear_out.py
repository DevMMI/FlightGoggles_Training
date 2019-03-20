import os
trial_storage = "gate_results/trial_storage.txt"

def main():
    f = open(trial_storage, "r")
    lines = f.readlines()
    vals = []
    for l in lines:
        print(int(l.strip('\n')))
        vals.append(int(l.strip('\n')))

    if(vals[1] < vals[0]):
        print("vals error")
        return(-1)

    os.system('python clear_values.py {} {}'.format(vals[0], vals[1]))

main()
