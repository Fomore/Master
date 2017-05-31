import cPickle
import cv2
import numpy as np

f = open('/home/falko/Uni/Master/Bilder/SynthEyes_data/f01/f01_42_-0.0000_-0.1963.pkl', 'rb')
db = cPickle.load(f)
f.close()

#print(db['ldmks']['ldmks_iris_2d'])

image = cv2.imread('/home/falko/Uni/Master/Bilder/SynthEyes_data/f01/f01_42_-0.0000_-0.1963.png')
#(x_centre,y_centre),(minor_axis,major_axis),angle

mImagePaths = cv2.glob('/home/falko/Uni/Master/Bilder/SynthEyes_data/*.png')
print(mImagePaths)

iris = np.array(db['ldmks']['ldmks_iris_2d']).astype(float32)
pupil = np.array(db['ldmks']['ldmks_pupil_2d']).astype(float32)
print(iris)
ellipse_i = cv2.fitEllipse(iris)
ellipse_p = cv2.fitEllipse(pupil)
cv2.ellipse(image,ellipse_i,(0,255,0))
cv2.ellipse(image,ellipse_p,(0,0,255))
print(ellipse_i)
print(ellipse_p)
for i in range(len(iris)):
    image[int(round(iris[i,1])),int(round(iris[i,0]))] = (255,255,0)
for i in range(len(pupil)):
    image[int(round(pupil[i,1])),int(round(pupil[i,0]))] = (255,0,255)

cv2.imwrite('TestAuge.png',image)
cv2.imshow("Image", image)
cv2.waitKey(300)
cv2.destroyAllWindows()
