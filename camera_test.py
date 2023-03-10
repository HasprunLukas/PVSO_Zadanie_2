from ximea import xiapi
import cv2
import numpy as np


# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((5 * 7, 3), np.float32)
objp[:, :2] = np.mgrid[0:7, 0:5].T.reshape(-1, 2)
# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.


def hough_trans_circle(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    gray = cv2.medianBlur(gray, 5)

    rows = gray.shape[0]
    # print(rows)
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 2,
                              param1=200, param2=40,
                              minRadius=0, maxRadius=150)

    # print(circles)
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            center = (i[0], i[1])
            # circle center
            cv2.circle(image, center, 1, (0, 100, 100), 3)
            # circle outline
            radius = i[2]
            cv2.circle(image, center, radius, (255, 0, 255), 3)

    cv2.imshow("detected circles", image)


def get_chessboard(file, counter):
    cv2.imwrite('chessboard' + str(counter) + '.jpg', file)


def open_image(path):
    img = cv2.imread(path)
    cv2.imshow('test', img)


def print_intrinsic_properties(gray):
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None)
    fx = mtx[0][0]
    fy = mtx[1][1]
    cx = mtx[0][2]
    cy = mtx[1][2]
    print("fx : " + str(fx))
    print("fy : " + str(fy))
    print("cx : " + str(cx))
    print("cy : " + str(cy))
    print(mtx)


def chess_function(names):
    for name in names:
        img = cv2.imread(name)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (5, 7), None)
        # If found, add object points, image points (after refining them)
        # print(ret)
        if ret == True:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)
            # Draw and display the corners
            cv2.drawChessboardCorners(img, (7, 5), corners2, ret)
            cv2.imshow(name, img)
            print_intrinsic_properties(gray)
            cv2.waitKey(500)


def main():
    # create instance for first connected camera
    cam = xiapi.Camera()

    # start communication
    # to open specific device, use:
    # cam.open_device_by_SN('41305651')
    # (open by serial number)
    print('Opening first camera...')
    cam.open_device()
    # settings
    cam.set_exposure(10000)
    cam.set_param("imgdataformat", "XI_RGB32")
    cam.set_param("auto_wb", 1)

    print('Exposure was set to %i us' % cam.get_exposure())

    # create instance of Image to store image data and metadata
    img = xiapi.Image()

    # start data acquisition
    print('Starting data acquisition...')
    cam.start_acquisition()

    counter = 0
    key = cv2.waitKey()
    chess_images = ['chessboard0.jpg', 'chessboard1.jpg', 'chessboard2.jpg', 'chessboard3.jpg']
    while key != ord('q'):
        cam.get_image(img)
        image = img.get_image_data_numpy()
        # image = cv2.resize(image, (240, 240))
        # image = cv2.imread('chessboard_BKP.jpg')

        scale_percent = 30  # percent of original size
        width = int(image.shape[1] * scale_percent / 100)
        height = int(image.shape[0] * scale_percent / 100)
        dim = (width, height)
        image = cv2.resize(image, dim, interpolation=cv2.INTER_AREA)

        cv2.imshow("test", image)
        # if key == ord('c'):
        #     get_chessboard(image, counter)
        #     counter = counter + 1
        # if key == ord('p'):
        #     open_image('chessboard_BKP.jpg')
        if key == ord('t'):
            chess_function(chess_images)
        if key == ord('c'):
            hough_trans_circle(image)
        key = cv2.waitKey()

    # stop data acquisition
    print('Stopping acquisition...')
    cam.stop_acquisition()

    # stop communication
    cam.close_device()

    print('Done.')


if __name__ == "__main__":
    main()
