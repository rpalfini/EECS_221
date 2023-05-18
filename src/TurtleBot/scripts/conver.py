


def convert_index_to_real(index):
    return index*0.05 + -10

def convert_real_to_index(real):
    # return -10 + int(round(real/0.05)) #This looks incorrect...
    point = abs(real-(-10))
    return int(round(point/0.05))