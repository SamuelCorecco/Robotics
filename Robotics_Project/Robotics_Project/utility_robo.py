def swap_dir_array(arr, dir):

    N = arr[0]
    S = arr[1]
    E = arr[2]
    O = arr[3]
    
    match dir:
        case "N":
            return arr
        case "S":
            tmp = arr[1]
            arr[1] = arr[0]
            arr[0] = tmp
            tmp = arr[3]
            arr[3] = arr[2]
            arr[2] = tmp
            return arr
        case "E":
            arr[0] = O
            arr[1] = E
            arr[2] = N
            arr[3] = S
            return arr
        case "O":
            tmp = arr[3]
            arr[3] = arr[0]
            arr[0] = tmp
            tmp = arr[2]
            arr[2] = arr[1]
            arr[1] = tmp
            tmp = arr[3]
            arr[3] = arr[2]
            arr[2] = tmp
            return arr
        