import numpy as np
import cv2
import numpy as np
import matplotlib.pyplot as plt
def merge_horizontal_lines(lines, horizontal_threshold=5, merge_threshold=5, top_threshold=15):

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
                last_line = [(last_x1, last_y1, x2, y2)]
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
    (3,1,1): [1,1,1,1],     # YES forward YES right YES left      WE have 3 lines, 1 in the first 1/6 and 1 in the last 1/6
    (2,0,1): [1,1,0,1],     # YES forward YES right NO  left      WE have 2 lines, 0 in the first 1/6 and 1 in the last 1/6
    (2,1,0): [1,1,1,0],     # YES forward NO  right YES left      WE have 2 lines, 1 in the first 1/6 and 0 in the last 1/6
    (2,1,1): [0,1,1,1],     # NO  forward YES right YES left      WE have 2 lines, 1 in the first 1/6 and 1 in the last 1/6
    (1,1,1): [0,1,1,1],     # NO  forward YES right YES left      WE have 2 lines, 1 in the first 1/6 and 1 in the last 1/6
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
    (1, 1, 1): "incrocio destra e sinistra",
    (1,1,0): "curva left",
    (1,0,1): "curva right"
}




# def select_type(img,threshold_first_last=10, show=True,debug=False):

#     gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#     gray = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY)[1]
#     edges = cv2.Canny(gray, 50, 200)
#     kernel = np.ones((2,2),np.uint8)
#     edges = cv2.dilate(edges,kernel,iterations = 1)

#     lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=20, minLineLength=5, maxLineGap=5)

#     # plot all lines
#     # image_tmp2 = img.copy()
#     # for line in lines:
#     #     x1, y1, x2, y2 = line[0]
#     #     cv2.line(image_tmp2, (x1, y1), (x2, y2), (0, 255, 0), 2)
#     # plt.imshow(cv2.cvtColor(image_tmp2, cv2.COLOR_BGR2RGB))
#     # plt.title("All lines")
#     # plt.show()
    
#     merged_lines = merge_horizontal_lines(lines)


#     first_line = 0
#     last_line = 0
#     number_of_lines = len(merged_lines)
#     for line in merged_lines:
#         x1, y1, x2, y2 = line
#         if x1 < threshold_first_last:
#             first_line += 1
#         if x2 > img.shape[1]-threshold_first_last:
#             last_line += 1
    
#     if first_line > 1:
#         first_line = 1
#     if last_line > 1:
#         last_line = 1
#     if number_of_lines > 3:
#         number_of_lines = 3


# prima 10
def select_type(img,threshold_first_last=10, show=True,debug=False):

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY)[1]
    edges = cv2.Canny(gray, 50, 200)
    kernel = np.ones((2,2),np.uint8)
    edges = cv2.dilate(edges,kernel,iterations = 1)

    lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=20, minLineLength=5, maxLineGap=5)
    
    merged_lines, not_horizontal_lines = merge_horizontal_lines(lines)

    vertical_lines = []
    diagonal_lines = []
    for line in not_horizontal_lines:
        x1, y1, x2, y2 = line[0]
        diff_x = x2 - x1
        if abs(diff_x) < threshold_first_last:
            vertical_lines.append(line)
        else:
            diagonal_lines.append(line)

    # Ghet the line in the borders of the image and the number of lines
    # borders and horizontal lines are useful to understand if there is a open path
    # number of lines is useful to understand the type of intersection and the front

    first_line = 0
    last_line = 0
    number_of_lines = len(merged_lines)
    tmp_line_left = None
    tmp_line_right = None
    for line in merged_lines:
        x1, y1, x2, y2 = line
        if x1 < threshold_first_last:
            first_line += 1
            tmp_line_left = line
        if x2 > img.shape[1]-threshold_first_last:
            last_line += 1
            tmp_line_right = line
    
    #if first_line == 0 and last_line >= 1:
    #    x_a, x_b , y_a, y_b = tmp_line_right
    #    for line in merged_lines:
    #        x1, y1, x2, y2 = line
    #        if y1 > y_a or y2 > y_b:
    #            last_line = 0
    #            break
    #elif first_line == 1 and last_line >= 0:
    #    x_a, x_b , y_a, y_b = tmp_line_left
    #    for line in merged_lines:
    #        x1, y1, x2, y2 = line
    #        if y1 > y_a or y2 > y_b:
    #            first_line = 0
    #            break
    

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
            if x1 < img.shape[0]/2 and x2 > img.shape[0]/2:
                lines_middle.append(line)

        
        longest_line = 0
        longest_line_line = None
        for line in lines_middle:
            x1, y1, x2, y2 = line
            if x2-x1 > longest_line:
                longest_line = x2-x1
                longest_line_line = line

        is_centered = False
        # check if exist non horizontal lines that are near x_1 y_1 and x_2 y_2 of longest_line_line
        flag_left = False
        flag_right = False
        if longest_line_line is not None:
            for line in diagonal_lines:
                x1, y1, x2, y2 = line[0]
                # check if any the two points of the line are near the two points of the longest line
                # left
                if abs(x1-longest_line_line[0]) < 5 and abs(y1-longest_line_line[1]) < 5:
                    flag_left = True
                if abs(x1-longest_line_line[2]) < 5 and abs(y1-longest_line_line[3]) < 5:
                    flag_left = True

                # right
                if abs(x2-longest_line_line[0]) < 5 and abs(y2-longest_line_line[1]) < 5:
                    flag_right = True
                if abs(x2-longest_line_line[2]) < 5 and abs(y2-longest_line_line[3]) < 5:
                    flag_right = True   

            
        is_centered = flag_left and flag_right
        
        flag_case_new = False
        # img.shape[1] > 300 to avoid wen we return in exploration and we have smaller image
        if img.shape[1] > 300 and longest_line > img.shape[1] * 13/20:
            print("Voglio morire")
            if flag_left and not flag_right:
                number_of_lines = 1
                first_line = 1
                last_line = 0
                flag_case_new = True
            if flag_right and not flag_left:
                number_of_lines = 1
                first_line = 0
                last_line = 1
                flag_case_new = True
        

        if (longest_line < img.shape[1] * 5/10 or not is_centered) and (not flag_case_new):
            number_of_lines = 1000


        if debug:
            print("len_middle_line", longest_line)
            print("FORWARD")



    # Select case
    
    key_case = (number_of_lines, first_line, last_line)
    name = "not case"
    value = [0,0,0,0]

    if key_case not in CASES:
        #print("Case not found")
        value = [0,0,0,0]
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
        
        for line in diagonal_lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(img, (x1, y1), (x2, y2), (255, 0, 255), 3)
            

        cv2.putText(img, name, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_AA)
        cv2.imshow("camera style", img)
        cv2.waitKey(1)

    return value
    



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

    if False:
        # plot the merged lines
        for line in merged_lines:
            x1, y1, x2, y2 = line
            cv2.line(img_tmp, (x1, y1), (x2, y2), (0, 0, 255), 2)
        plt.imshow(cv2.cvtColor(img_tmp, cv2.COLOR_BGR2RGB))
        plt.title('Merged Lines')
        plt.show()

        # show the image with the lines with cv2.imshow
        for line in merged_lines:
            x1, y1, x2, y2 = line
            cv2.line(img_tmp, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.imshow("Image with lines", img_tmp)
        cv2.waitKey(1)



# NOTE Debug work on jupiter notebook 

def check_if_sample(img, debug=False):
    # black and white

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    gray = cv2.blur(gray, (3, 3))
    low_threshold = 185
    high_threshold = 195
    mask = cv2.inRange(gray, low_threshold, high_threshold)

    if debug:
        plt.imshow(mask, cmap='gray')
        plt.title("Mask")
        plt.show()


    edges = cv2.Canny(mask, 50, 200)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 100, minLineLength=150, maxLineGap=50)

    # if no lines detected return False and -1
    if lines is None:
        return False, -1

    if debug:
        image_tmp = img.copy()
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(image_tmp, (x1, y1), (x2, y2), (0, 255, 0), 2)

        plt.imshow(cv2.cvtColor(image_tmp, cv2.COLOR_BGR2RGB))
        plt.title('ALL Lines')
        plt.show()

        print("Number of lines", len(lines))
    
    # get only horizontal lines
    horizontal_lines = []
    for line in lines:
        x1, y1, x2, y2 = line[0]
        diff_y = y2 - y1
        if abs(diff_y) < 5:
            horizontal_lines.append(line)

    if debug:
        # plot the lines
        for line in horizontal_lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

        plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        plt.title('Detected Lines')
        plt.show()



    # check if a line have y between 145 and 155

    middle_y = img.shape[0] * 65/100
    Sample_line = False
    min_diff = 100000
    for line in horizontal_lines:
        x1, y1, x2, y2 = line[0]
        if y1 > middle_y-4 and y1 < middle_y+4:
            if min_diff > abs((y2-y1)/(x2-x1)):
                min_diff = ((y2-y1)/(x2-x1))
            Sample_line = True
        if y2 > middle_y-4 and y2 < middle_y+4:
            Sample_line = True
            if min_diff > abs((y2-y1)/(x2-x1)):
                min_diff = ((y2-y1)/(x2-x1))

    ###########################
    ###########################
    ###########################
    ###########################
    ###########################
    
    if True:
        image_tmp = img.copy()
        for line in horizontal_lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(image_tmp, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.line(image_tmp, (0, int(middle_y-5)), (img.shape[1], int(middle_y-5)), (255, 0, 0), 2)
        cv2.line(image_tmp, (0, int(middle_y+5)), (img.shape[1], int(middle_y+5)), (255, 0, 0), 2)
        cv2.imshow("Image with lines", image_tmp)
        cv2.waitKey(1)


    if Sample_line:
        return Sample_line , min_diff
    
    
    # now we can check the difference between y1 and y2 for the line with biggest y2
    lines_sorted = sorted(lines, key=lambda x: x[0][3])
    x1, y1, x2, y2 = lines_sorted[-1][0]

    # plot the lines_sorted[-1]
    if debug:
        image_tmp = img.copy()
        cv2.line(image_tmp, (x1, y1), (x2, y2), (0, 255, 0), 2)
        plt.imshow(cv2.cvtColor(image_tmp, cv2.COLOR_BGR2RGB))
        plt.title('Line with biggest y2')
        plt.show()

    min_diff = ((y2-y1)/(x2-x1))

    return Sample_line ,  min_diff 




def get_pendence(img):

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    gray = cv2.blur(gray, (3, 3))
    low_threshold = 185
    high_threshold = 195
    mask = cv2.inRange(gray, low_threshold, high_threshold)
    edges = cv2.Canny(mask, 50, 200)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 100, minLineLength=150, maxLineGap=50)

    edges = cv2.Canny(mask, 50, 200)
    # # make endge more bigger
    # kernel = np.ones((2,2),np.uint8)
    # edges = cv2.dilate(edges,kernel,iterations = 1)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 100, minLineLength=200, maxLineGap=100)

    if lines is None:
        kernel = np.ones((2,2),np.uint8)
        edges = cv2.dilate(edges,kernel,iterations = 1)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 100, minLineLength=150, maxLineGap=50)
        if lines is None:
            return None
    
    # add 0.00001 to avoid division by 0
    lines = [line for line in lines if abs((line[0][3]-line[0][1])/(line[0][2]-line[0][0]+ 0.00001)) < 0.08]

    # remove all line wit y1 and y2 > 5/6 of the image
    lines = [line for line in lines if line[0][1] < img.shape[0]*5/6 and line[0][3] < img.shape[0]*5/6]
    # remove line with y1 and y2 < 1/6 of the image
    lines = [line for line in lines if line[0][1] > img.shape[0]*1/6 and line[0][3] > img.shape[0]*1/6]
    if len(lines) == 0:
        return None
    
    # plot the lines
    if True:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(img, (x1, y1), (x2, y2), (255, 0, 255), 2)
        cv2.imshow("Image with lines", img)
        cv2.waitKey(1)
    
    # get the line with the smallest y
    lines_sorted = sorted(lines, key=lambda x: x[0][3])
    x1, y1, x2, y2 = lines_sorted[0][0]
    pendence = (y2-y1)/(x2-x1)
    if len(lines) > 1:
        x3, y3, x4, y4 = lines_sorted[1][0]
        pendence2 = (y4-y3)/(x4-x3)
        if pendence*pendence2 < 0:
            return None

    return pendence