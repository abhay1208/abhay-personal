# **Finding Lane Lines on the Road** 

### 1. Describe your pipeline. As part of the description, explain how you modified the draw_lines() function.

My pipeline consisted of 5 steps. 

1. Read the colored image and convert into grayscale
2. Apply gaussian blurring of 5x5 kernel window to smooth out the  grayscale image
3. Use Canny edge detecion to find edges in the image
4. Apply masking using a region of interest that contains the lanes we are interested in
5. Use hough transform to detected lines from the edges in the region of interest


In order to draw a single line on the left and right lanes, I modified the draw_lines() function by separating out positive and negative slope lines. I removed the outliers and took an average of the slope and y-intercept of the lines for both positive and negative lines. Then I found the intersection with the bottom row of the image and region of interest to draw both lane lines.

The output test images are saved in the test_images_output folder. The output of the videos is saved in the test_videos_output folder


### 2. Potential shortcomings with your current pipeline


One potential shortcoming would be what would happen when the lane lines are not straight. This approach only works when the lane lines are straight which is not going to be true around turns.

Another shortcoming could be the region of interest doesn't contain the lanes we are interested in. Although, I did not run into this issue with the given test images and videos, but I certainly see that as a possibility.


### 3. Possible improvements

A possible improvement would be to further refine the tunable parameters for a wider range of test images and videos.

Another potential improvement could be to find arc of the circle instead of finding lines that would give better results with lane finding around turns.
