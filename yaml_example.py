import yaml

stream = open("gate_locations_1.yaml", "r")
docs = yaml.load_all(stream)
for doc in docs:
    for k,v in doc.items():
        for l, m in v.items():
            print k, ":"#, m, "\n"
            for vals in m:
                # for v in vals:
                #     print(str(v)+",")
                # three values in vals
                print(str(vals)+"\n")
                # strs = vals.replace('[','').split('],')
                # lists = [map(int, s.replace(']','').split(',')) for s in strs]
                # for l in lists:
                #     print("l : {}".format(l))


        print "\n\n",
    print "\n\n",
