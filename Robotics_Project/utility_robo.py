def swap_dir_array(arr, dir):

    N = arr[0]
    S = arr[1]
    E = arr[2]
    O = arr[3]
    
    match dir:
        case "N":
            # NOTA: ho fatto io un casino e ho invertito E e O quindi sry
            # cambio poi se abbiamo tempo
            arr[0] = N
            arr[1] = S
            arr[2] = O
            arr[3] = E
            return arr
        case "S":
            arr[0] = S
            arr[1] = N
            arr[2] = E
            arr[3] = O
            return arr
        case "E":
            arr[0] = E
            arr[1] = O
            arr[2] = N
            arr[3] = S
            return arr
        case "O":
            arr[0] = O
            arr[1] = E
            arr[2] = S
            arr[3] = N
            return arr


# We have direction N S E O
# we need to compute if we need to go right or left
# Ex: if my direction is N and I want to go to E, I need to go right
# NOTE: we can't have the opposite direction only right or left
# return 1 if right, -1 if left
def get_angle(next_dir, curr_dir):
    match curr_dir:
        case "N":
            match next_dir:
                case "E":
                    return 1.0
                case "O":
                    return -1.0
        case "S":
            match next_dir:
                case "E":
                    return -1.0
                case "O":
                    return 1.0
        case "E":
            match next_dir:
                case "S":
                    return 1.0
                case "N":
                    return -1.0
        case "O":
            match next_dir:
                case "S":
                    return -1.0
                case "N":
                    return 1.0