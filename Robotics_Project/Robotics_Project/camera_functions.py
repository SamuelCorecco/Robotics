import numpy as np
import cv2
import numpy as np
import matplotlib.pyplot as plt
def merge_horizontal_lines(lines, horizontal_threshold=10, merge_threshold=5, top_threshold=10):

    # check if is empty
    if lines is None or len(lines) == 0:
        return []
    
    # remove top line (can by the "sky and are not good for us")
    lines = [line for line in lines if line[0][1] > top_threshold]

    # We choose only horizontal lines or lines that are close to horizontal
    # we need only horizontal lines for the task
    horizontal_lines = []
    not_horizontal_lines = []
    for line in lines:
        x1, y1, x2, y2 = line[0]
        diff_y = y2 - y1
        if abs(diff_y) < horizontal_threshold:
            horizontal_lines.append(line)
        else:
            not_horizontal_lines.append(line)
    horizontal_lines.sort(key=lambda x: x[0][0])

    not_horizontal_lines.sort(key=lambda x: x[0][0])
    
    # merge lines
    # this part concatenate lines that are close to each other
    merged_lines = []
    if horizontal_lines:
        last_line = horizontal_lines[0]
        for line in horizontal_lines[1:]:
            x1, y1, x2, y2 = line[0]
            last_x1, last_y1, last_x2, last_y2 = last_line[0]
            if x1 - last_x2 < merge_threshold:
                x2_new = max(x2, last_x2)
                y2_new = max(y2, last_y2)
                x1_new = min(x1, last_x1)
                y1_new = min(y1, last_y1)
                last_line = [(x1_new, y1_new, x2_new, y2_new)]
            else:
                merged_lines.extend(last_line)
                last_line = line
        merged_lines.extend(last_line)
    return [merged_lines, not_horizontal_lines]


# we have 5 cases
#  NO avanti NO destra NO sinistra 
#  SI avanti SI destra SI sinistra
#  SI aventi SI destra NO sinistra
#  SI avanti NO destra SI sinistra
#  NO avanti SI destra SI sinistra

# curve cases
#  NO avanti SI destra NO sinistra
#  NO avanti NO destra SI sinistra

# we use for the key that are horizontal lines 
# we have 3 values for each key :   tot number of lines
#                                   number of lines in the first 1/6, 
#                                   number of lines in the last 1/6, 

# return North, sud, est, ovest
# north = forward
# south = true
# east = right
# west = left


CASES = {
    (1,0,0): [0,1,0,0],     # NO  forward NO  right NO  left      WE have 1 line , 0 in the first 1/6 and 0 in the last 1/6
    (3,1,1): [1,1,0,1],     # YES forward YES right YES left      WE have 3 lines, 1 in the first 1/6 and 1 in the last 1/6
    (2,0,1): [1,1,1,0],     # YES forward YES right NO  left      WE have 2 lines, 0 in the first 1/6 and 1 in the last 1/6
    (2,1,0): [1,1,0,1],     # YES forward NO  right YES left      WE have 2 lines, 1 in the first 1/6 and 0 in the last 1/6
    (2,1,1): [0,1,1,1],     # NO  forward YES right YES left      WE have 2 lines, 1 in the first 1/6 and 1 in the last 1/6
    (1,1,0): [0,1,1,0],     # NO  forward YES right NO  left      WE have 1 line , 1 in the first 1/6 and 0 in the last 1/6
    (1,0,1): [0,1,0,1]      # NO  forward NO  right YES left      WE have 1 line , 0 in the first 1/6 and 1 in the last 1/6
}

# Just for debug
CASES_NAME = {  
    (1,0,0): "punto morto",
    (3,1,1): "incrocio con tutte le direzioni",
    (2,0,1): "incrocio avanti e destra",
    (2,1,0): "incrocio avanti e sinistra",
    (2,1,1): "incrocio destra e sinistra",
    (1,1,0): "curva right",
    (1,0,1): "curva left"
}




def select_type(img,threshold_first_last=10, show=True,debug=False):

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY)[1]
    edges = cv2.Canny(gray, 50, 200)
    kernel = np.ones((2,2),np.uint8)
    edges = cv2.dilate(edges,kernel,iterations = 1)

    lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=20, minLineLength=5, maxLineGap=5)

    # plot all lines
    # image_tmp2 = img.copy()
    # for line in lines:
    #     x1, y1, x2, y2 = line[0]
    #     cv2.line(image_tmp2, (x1, y1), (x2, y2), (0, 255, 0), 2)
    # plt.imshow(cv2.cvtColor(image_tmp2, cv2.COLOR_BGR2RGB))
    # plt.title("All lines")
    # plt.show()
    
    merged_lines = merge_horizontal_lines(lines)


    first_line = 0
    last_line = 0
    number_of_lines = len(merged_lines)
    for line in merged_lines:
        x1, y1, x2, y2 = line
        if x1 < threshold_first_last:
            first_line += 1
        if x2 > img.shape[1]-threshold_first_last:
            last_line += 1
    
    if first_line > 1:
        first_line = 1
    if last_line > 1:
        last_line = 1
    if number_of_lines > 3:
        number_of_lines = 3



def select_type(img,threshold_first_last=10, show=True,debug=False):

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY)[1]
    edges = cv2.Canny(gray, 50, 200)
    kernel = np.ones((2,2),np.uint8)
    edges = cv2.dilate(edges,kernel,iterations = 1)

    lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=20, minLineLength=5, maxLineGap=5)
    
    merged_lines, not_horizontal_lines = merge_horizontal_lines(lines)


    # Ghet the line in the borders of the image and the number of lines
    # borders and horizontal lines are useful to understand if there is a open path
    # number of lines is useful to understand the type of intersection and the front

    first_line = 0
    last_line = 0
    number_of_lines = len(merged_lines)
    for line in merged_lines:
        x1, y1, x2, y2 = line
        if x1 < threshold_first_last:
            first_line += 1
        if x2 > img.shape[1]-threshold_first_last:
            last_line += 1
    
    if first_line > 1:
        first_line = 1
    if last_line > 1:
        last_line = 1
    if number_of_lines > 3:
        number_of_lines = 3



    # special case for curve 
    # check the smallest y of the line with a x_1 < threshold_first_last and the smallest y of the line with a x_2 > img.shape[1]-threshold_first_last
    # we have only one line and we have to check if the line is more on the left or on the right

    if number_of_lines ==1 and first_line == 0 and last_line == 0:
        y_middle_line = 100000

        for line in merged_lines:
            x1, y1, x2, y2 = line
            y_middle_line = min(y_middle_line, (y1+y2)/2)
            

        max_y_left = 100000
        max_y_right = 100000
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x1 < threshold_first_last:
                max_y_left = min(max_y_left, y1)
            elif x2 > img.shape[1]-threshold_first_last:
                max_y_right = min(max_y_right, y2)

        if y_middle_line-threshold_first_last > max_y_left or y_middle_line-threshold_first_last > max_y_right:
            if max_y_left < max_y_right:
                number_of_lines = 1
                first_line = 1
                last_line = 0
            if max_y_left > max_y_right:
                number_of_lines = 1
                first_line = 0
                last_line = 1



    # check iff we don't have left or right lines
    # IS a forward that can be see as a dead end but not in the next 
    if first_line == 0 and  last_line == 0:
        lines_middle = []
        # if x_1 < middle of the image and x_2 > middle of the image
        for line in merged_lines:
            x1, y1, x2, y2 = line
            if x1 < img.shape[1]/2 and x2 > img.shape[1]/2:
                lines_middle.append(line)

        longest_line = 0
        for line in lines_middle:
            x1, y1, x2, y2 = line
            if x2-x1 > longest_line:
                longest_line = x2-x1

        if longest_line < 250:
            number_of_lines = 1000

        if debug:
            print("len_middle_line", longest_line)
            print("FORWARD")



    # Select case
    
    key_case = (number_of_lines, first_line, last_line)
    name = "not case"
    value = [0,0,0,0]

    if key_case not in CASES:
        print("Case not found")
    else:
        value = CASES[key_case]
        name = CASES_NAME[key_case]
        
    
    if debug:
        print("first_1_4: ", first_line)
        print("last_1_4: ", last_line)
        print("total lines: ", len(merged_lines))

        print("CASE", value)
        print("CASE NAME", name)

    if show:
        for line in merged_lines:
            x1, y1, x2, y2 = line
            cv2.line(img, (x1, y1), (x2, y2), (255, 0, 0), 3)
        # plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        # plt.title(name)
        # plt.show()
        cv2.imshow("Image with lines", img)
        cv2.waitKey(1)
    



def plot_img_with_line(img):
    img_tmp = img.copy()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 200)
    kernel = np.ones((5,5),np.uint8)
    edges = cv2.dilate(edges,kernel,iterations = 1)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 100, minLineLength=50, maxLineGap=250)
    merged_lines, _ = merge_horizontal_lines(lines)

    # plot the merged lines
    for line in merged_lines:
        x1, y1, x2, y2 = line
        cv2.line(img_tmp, (x1, y1), (x2, y2), (255, 0, 0), 3)
    plt.imshow(cv2.cvtColor(img_tmp, cv2.COLOR_BGR2RGB))
    plt.title('Merged Lines')
    plt.show()


    
def plot_img_with_line2(img, debug=False):
    img_tmp = img.copy()

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY)[1]
    edges = cv2.Canny(gray, 50, 200)
    kernel = np.ones((2,2),np.uint8)
    edges = cv2.dilate(edges,kernel,iterations = 1)

    lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=20, minLineLength=5, maxLineGap=5)


    if debug:
        # plot edges
        plt.imshow(edges, cmap='gray')
        plt.title("Edges")
        plt.show()
            
        # plot the lines
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(img_tmp, (x1, y1), (x2, y2), (0, 255, 0), 2)

        plt.imshow(cv2.cvtColor(img_tmp, cv2.COLOR_BGR2RGB))
        plt.title('Detected Lines')
        plt.show()

    merged_lines, _ = merge_horizontal_lines(lines)

    # plot the merged lines
    #for line in merged_lines:
    #    x1, y1, x2, y2 = line
    #    cv2.line(img_tmp, (x1, y1), (x2, y2), (0, 0, 255), 2)
    #plt.imshow(cv2.cvtColor(img_tmp, cv2.COLOR_BGR2RGB))
    #plt.title('Merged Lines')
    #plt.show()

    # show the image with the lines with cv2.imshow
    for line in merged_lines:
        x1, y1, x2, y2 = line
        cv2.line(img_tmp, (x1, y1), (x2, y2), (255, 0, 0), 2)
        cv2.imshow("Image with lines", img_tmp)
    cv2.waitKey(1)
