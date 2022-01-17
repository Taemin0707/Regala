#!/usr/bin/env python3

import rospy
import numpy as np

import cv2

surf = cv2.SIFT_create()

left_img = cv2.imread('/home/taemin/real_left.jpg', 0)
right_img = cv2.imread('/home/taemin/real_right.jpg', 0)

# Gray Images for detecting features
# gray_left_img = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
# gray_right_img = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)

l_keypoints, l_desc = surf.detectAndCompute(left_img, None)
r_keypoints, r_desc = surf.detectAndCompute(right_img, None)

# FLANN parameters
FLANN_INDEX_KDTREE = 0
index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
search_params = dict(checks=50)

flann = cv2.FlannBasedMatcher(index_params, search_params)

# Matching descriptors
matches = flann.knnMatch(l_desc, r_desc, k=2)

# print("+++++++++++++++++++++++++++++++++++++++++++++++")
# print(matches[0][0].queryIdx)
# print(matches[0][1].trainIdx)

matchesMask = [[0,0] for i in range(len(matches))] 
good_matches = []

for i, (m, n) in enumerate(matches): 
    if m.distance < 0.40 * n.distance: 
        matchesMask[i] = [1, 0]
        good_matches.append(m)


# for i, (m, n) in enumerate(matches):
#     if m.distance < 0.7 * n.distance:
#         good_matches.append(i)
        
print(good_matches[0].queryIdx)
print(l_keypoints[good_matches[0].queryIdx].pt)

l_points = []
r_points = []

for i in range(len(good_matches)):
    # print(l_keypoints[good_matches[i]].pt)
    # print(r_keypoints[good_matches[i]].pt)
    l_points.append(l_keypoints[good_matches[i].queryIdx].pt)
    r_points.append(r_keypoints[good_matches[i].trainIdx].pt)

l_points = np.array(l_points)
r_points = np.array(r_points)

homography, _ = cv2.findHomography(r_points, l_points)
result = cv2.warpPerspective(right_img, homography, (1920, 1080))

print(homography)
# print(r_points)


# print(l_keypoints[0].pt)

draw_params = dict(matchColor = (0, 255, 0), \
                   singlePointColor = (255,0,0), \
                   matchesMask = matchesMask, \
                   flags = 0)


img_draw = cv2.drawMatchesKnn(left_img, l_keypoints, right_img, r_keypoints, \
                              matches, None, **draw_params)

output_shape = (left_img.shape[1] + right_img.shape[1], left_img.shape[0])
result = cv2.warpPerspective(right_img, homography, (output_shape))
# result[0:left_img.shape[0], 0:left_img.shape[1]] = left_img
result[0:left_img.shape[0], 0:300] = left_img[0:left_img.shape[0], 0:300]

# for point in r_points:
#     cv2.circle(right_img, (int(point[0]), int(point[1])), 7, (255, 0, 0), -1)

cv2.imshow('Test', result)
cv2.waitKey()
cv2.destroyAllWindows()

# for i in range(len(matches)):
#     print()

# if self.saved_homo_matrix is None:
#     # try:
#     left_ret, left_corners = cv2.findChessboardCorners(cv_left_image, (self.cb_row, self.cb_col), None)
#     right_ret, right_corners = cv2.findChessboardCorners(cv_right_image, (self.cb_row, self.cb_col), None)

#     print(left_ret, right_ret)

#     self.left_corners = left_corners
#     self.right_corners = right_corners