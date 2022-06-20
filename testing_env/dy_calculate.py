#!/usr/bin/python3


from math import pi, sin, cos
import numpy as np
import matplotlib.pyplot as plt


def CalculateDeltY( palm_width, segment_ratio, max_span) -> None:
    # palm_width = palm_width
    # segment_ratio = segment_ratio
    # print(segment_ratio)
    num_segments = len(segment_ratio)
    # max_span = max_span
    object_width = 0.25 * max_span


    if num_segments == 2:
        finger_len = 0.5*(max_span - palm_width) / (segment_ratio[0] + segment_ratio[-1]* sin(120*pi/180))
        y_origin = 0.75* (finger_len*segment_ratio[-1] + ( (finger_len*(segment_ratio[0] + segment_ratio[-1]))**2 - palm_width**2)**0.5)
    elif num_segments == 3:
        finger_len = 0.5*(max_span - palm_width) / (segment_ratio[0] + segment_ratio[1] + segment_ratio[-1]* sin(120*pi/180))
        y_origin = 0.75* (finger_len*segment_ratio[-1] + ( (finger_len(segment_ratio[0] + segment_ratio[1] + segment_ratio[-1]))**2 - palm_width**2)**0.5)
    y_max = 0.5*object_width + (finger_len**2 - (0.5*(palm_width - object_width))**2)**0.5

    y_delta = y_max - y_origin

    return finger_len, y_delta


if __name__ == '__main__':

    palm_width = .31
    max_span = .9
    segment_ratio = []
    results = []
    distal = []
    # print(len(np.arange(0.1,0.52, 0.02)))
    for distal_ratio in np.arange(0.1,0.52, 0.01):
        # other_ratio = round(((1 - distal_ratio) / 2), 3)
        other_ratio = round(1 - distal_ratio, 3)
        segment_ratio.append([other_ratio, distal_ratio])
        distal.append(distal_ratio) 
    
    # print(len(segment_ratio))
    finger_length = []
    delta_y = []
    for segment in segment_ratio:

        # print(segment)
        fl, dy = CalculateDeltY(palm_width=palm_width, segment_ratio=segment, max_span=max_span)
        finger_length.append(fl)
        delta_y.append(dy)
    
    # print(results)

    plt.plot(distal , delta_y) 
    plt.xlabel('Percent of Finger which is the Distal')
    plt.ylabel('Theoretical astrisk score for the North Direction')
    plt.title('Varying the Distal Link Percentage (palm width 0.31 max span 0.9)')
    plt.show()



    # palm_width = np.arange(.05,0.32, 0.01)
    # max_span = .9
    # segment_ratio = [0.5,0.5]
    # results = []
    # # distal = 
    
    # # print(len(segment_ratio))
    # finger_length = []
    # delta_y = []
    # for palm in palm_width:

    #     # print(segment)
    #     fl, dy = CalculateDeltY(palm_width=palm, segment_ratio=segment_ratio, max_span=max_span)
    #     finger_length.append(fl)
    #     delta_y.append(dy)

    # plt.plot(palm_width , delta_y) 
    # plt.xlabel('Palm Width')
    # plt.ylabel('Theoretical astrisk score for the North Direction')
    # plt.title('Varying the palm width (2 Link 50/50 ratio max span 0.9)')
    # plt.show()
