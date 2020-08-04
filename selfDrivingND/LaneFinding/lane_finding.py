import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2
import math
import os
from moviepy.editor import VideoFileClip
from IPython.display import HTML
import matplotlib.pyplot as plt 

def read_image(image_name):
    return mpimg.imread(image_name)


def process_image(image):
    # NOTE: The output you return should be a color image (3 channel) for processing video below
    # TODO: put your pipeline here,
    # you should return the final output (image where lines are drawn on lanes)
    # Read the original image
    # image_color = mpimg.imread(image)
    # gray = grayscale(image_color)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Region of interest
    height = gray.shape[0]
    width = gray.shape[1]

    # Apply gaussian filtering on the image and do edge detection
    gray = gaussian_blur(gray, 5)
    edges = canny(gray, 50, 150)

    # Mask Region of interest
    vertices = np.array([[0,height], [width/2 - 20, height*0.6], [width/2+20, height*0.6], [width, height]], dtype=np.int32)
    masked_edges = region_of_interest(edges, [vertices])

    # # Apply hough transform to find lines
    theta = math.pi/180.0
    rho = 2
    threshold = 20
    minLineLength = 20
    maxLineGap = 10
    lines = hough_lines(masked_edges, rho, theta, threshold, minLineLength, maxLineGap)
    output = weighted_img(lines, image, 0.5, 1.5)
    return output

######### Helper Functions %%%%%%%%%%%%%
def grayscale(img):
    """Applies the Grayscale transform
    This will return an image with only one color channel
    but NOTE: to see the returned image as grayscale
    (assuming your grayscaled image is called 'gray')
    you should call plt.imshow(gray, cmap='gray')"""
    return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    # Or use BGR2GRAY if you read an image with cv2.imread()
    # return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
def canny(img, low_threshold, high_threshold):
    """Applies the Canny transform"""
    return cv2.Canny(img, low_threshold, high_threshold)

def gaussian_blur(img, kernel_size):
    """Applies a Gaussian Noise kernel"""
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

def region_of_interest(img, vertices):
    """
    Applies an image mask.
    
    Only keeps the region of the image defined by the polygon
    formed from `vertices`. The rest of the image is set to black.
    `vertices` should be a numpy array of integer points.
    """
    #defining a blank mask to start with
    mask = np.zeros_like(img)   
    
    #defining a 3 channel or 1 channel color to fill the mask with depending on the input image
    if len(img.shape) > 2:
        channel_count = img.shape[2]  # i.e. 3 or 4 depending on your image
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255
        
    #filling pixels inside the polygon defined by "vertices" with the fill color    
    cv2.fillPoly(mask, vertices, ignore_mask_color)
    
    #returning the image only where mask pixels are nonzero
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image


def draw_lines(img, lines, color=[255, 0, 0], thickness=4):
    """
    NOTE: this is the function you might want to use as a starting point once you want to 
    average/extrapolate the line segments you detect to map out the full
    extent of the lane (going from the result shown in raw-lines-example.mp4
    to that shown in P1_example.mp4).  
    
    Think about things like separating line segments by their 
    slope ((y2-y1)/(x2-x1)) to decide which segments are part of the left
    line vs. the right line.  Then, you can average the position of each of 
    the lines and extrapolate to the top and bottom of the lane.
    
    This function draws `lines` with `color` and `thickness`.    
    Lines are drawn on the image inplace (mutates the image).
    If you want to make the lines semi-transparent, think about combining
    this function with the weighted_img() function below
    """
  # initialize lists to hold line formula values
    b_left     = []  # b of left lines
    b_right    = []  # b of Right lines
    m_left = []  # m of Left lines
    m_right = []  # m of Right lines
    
    for line in lines:
        for x1,y1,x2,y2 in line:
            
            # get slope and intercept
            m = (y2-y1)/(x2-x1)
            b = y1 - x1*m
            
            # Remove outliers
            if m >= 0 and (m < 0.4 or m > 0.8):
                continue
            elif m < 0 and (m < -0.8 or m > -0.4):
                continue
                
            # Separate left and right lane lines
            if m > 0:
                m_left.append(m)
                b_left.append(b)
            else:
                m_right.append(m)
                b_right.append(b)
    
    # Get image shape and define y region of interest value
    imshape = img.shape
    y_max   = imshape[0] # lines initial point at bottom of image    
    y_min   = 350        # lines end point at top of ROI

    # Get mean of m and b
    mean_pos_m = mean(m_left)
    mean_neg_m = mean(m_right)
    mean_left_b     = mean(b_left)
    mean_right_b    = mean(b_right)

    # Generate x, y coordinates of the lane lines using mean m and b
    
    x1_Left = (y_max - mean_left_b)/mean_pos_m
    y1_Left = y_max
    x2_Left = (y_min - mean_left_b)/mean_pos_m
    y2_Left = y_min
    x1_Right = (y_max - mean_right_b)/mean_neg_m
    y1_Right = y_max
    x2_Right = (y_min - mean_right_b)/mean_neg_m
    y2_Right = y_min

    # define average left and right lines
    cv2.line(img, (int(x1_Left), int(y1_Left)), (int(x2_Left), int(y2_Left)), color, thickness) #avg Left Line
    cv2.line(img, (int(x1_Right), int(y1_Right)), (int(x2_Right), int(y2_Right)), color, thickness) #avg Right Line

def mean(my_list):
    return sum(my_list)/max(len(my_list), 1)

def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap):
    """
    `img` should be the output of a Canny transform.
        
    Returns an image with hough lines drawn.
    """
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
    line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    draw_lines(line_img, lines)
    return line_img

# Python 3 has support for cool math symbols.

def weighted_img(img, initial_img, α=0.8, β=1., γ=0.):
    """
    `img` is the output of the hough_lines(), An image with lines drawn on it.
    Should be a blank image (all black) with lines drawn on it.
    
    `initial_img` should be the image before any processing.
    
    The result image is computed as follows:
    
    initial_img * α + img * β + γ
    NOTE: initial_img and img must be the same shape!
    """
    return cv2.addWeighted(initial_img, α, img, β, γ)

def test_all_images():
    for image_name in os.listdir("test_images/"):
        folder_name = "test_images"
        save_file_name = image_name
        image_name = folder_name + "/" + image_name
        image = read_image(image_name)
        result = process_image(image)
        cv2.imwrite("test_images_output/" + save_file_name, result)
        plt.imshow(result)
        plt.show()

def test_single_image(image_name):
    image = read_image(image_name)
    out = process_image(image)
    plt.imshow(out)
    plt.show()

def test_video(video_name):
    video_file = video_name.split("/")
    white_output = "test_videos_output/"+ video_file[1]
    ## To speed up the testing process you may want to try your pipeline on a shorter subclip of the video
    ## To do so add .subclip(start_second,end_second) to the end of the line below
    ## Where start_second and end_second are integer values representing the start and end of the subclip
    ## You may also uncomment the following line for a subclip of the first 5 seconds
    ##clip1 = VideoFileClip("test_videos/solidWhiteRight.mp4").subclip(0,5)
    clip1 = VideoFileClip(video_name)
    white_clip = clip1.fl_image(process_image) #NOTE: this function expects color images!!
    white_clip.write_videofile(white_output, audio=False)

if __name__=="__main__":
    image_name = "test_images/solidYellowCurve.jpg"
    # test_single_image(image_name)
    test_all_images()
    # test_video("test_videos/solidYellowLeft.mp4")
    # test_video("test_videos/solidWhiteRight.mp4")

