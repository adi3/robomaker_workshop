#!/usr/bin/env python

import numpy as np

NUM_WORLDS = 10
COINS = ["monero", "dash", "litecoin", "etherium", "bitcoin"]
X_LOW = 0.06
X_HIGH = 0.15
# Y_LOW = -0.25
Y_MAX = 0.25
TABLE_HEIGHT = 0.715

INCLUDE = "\n\t\t<include>\n\
    \t<uri>model://COIN_NAME</uri>\n\
    \t<pose frame=''>TABLE_X TABLE_Y %.3f 0 -0 -1.6</pose>\n\
    </include>\n" % TABLE_HEIGHT


for i in range(NUM_WORLDS):
    # Number of coins to add to this scene
    num_coins = np.random.random_integers(1, len(COINS))
    # Indices from COINS for this scene
    coin_indices = np.random.randint(0, len(COINS), num_coins)
    # Names of coins for this scene
    coins = np.take(COINS, coin_indices)

    output = ""
    for c in coins:
        # Equal probability of X occuring anywhere in range
        x = np.random.uniform(X_LOW, X_HIGH)
        # Keeps coins within reachable range
        # y_clip = Y_MAX*(x - X_LOW)/(X_HIGH - X_LOW)
        y_clip = np.sqrt(np.square(X_HIGH) - np.square(x))
        # print(x)
        # print(y_clip)
        # print("---")
        # y = np.random.uniform(Y_LOW + y_clip, Y_HIGH - y_clip)
        y = np.random.uniform(-y_clip, y_clip)

        # Greater probability of Y occuring closer to zero
        # scale = np.random.normal(scale = 0.5)
        # while scale > 1.0:
        #     scale = np.random.normal(scale = 0.5)
        # y = y_clip * scale

        temp = INCLUDE.replace("COIN_NAME", c)
        temp = temp.replace("TABLE_X", str(np.around(x, 2)))
        temp = temp.replace("TABLE_Y", str(np.around(y, 2)))

        output += temp

    print("================")
    print(output)
    print("================")

    template = open("../worlds/template.world", "rt")
    world = open("../worlds/px100-%d.world" % i, "wt")

    for line in template:
        world.write(line.replace('<COINS>', output))

    world.close()
    template.close()
