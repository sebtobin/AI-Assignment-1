def heuristic(coordinate1_q, coordinate1_r, coordinate2_q, coordinate2_r):
    return (abs(coordinate1_q - coordinate2_q) +
            abs(coordinate1_r - coordinate2_r) +
            abs(coordinate1_q - coordinate2_q + coordinate1_r - coordinate2_r))/2