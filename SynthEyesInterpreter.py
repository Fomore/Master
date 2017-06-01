import cPickle
import cv2
import numpy as np
import fnmatch
import os

out = open('workfile.txt', 'w+')
#(x_centre,y_centre),(minor_axis,major_axis),angle
for root, dirnames, filenames in os.walk('/home/falko/Uni/Master/Bilder/SynthEyes_data/'):
    for filename in fnmatch.filter(filenames, '*.png'):
        path = os.path.join(root, filename);
        path2 = path[:-2]+'kl'
        f = open(path2, 'rb')
        db = cPickle.load(f)
        f.close()
        iris = np.array(db['ldmks']['ldmks_iris_2d']).astype(float32)
        pupil = np.array(db['ldmks']['ldmks_pupil_2d']).astype(float32)
        lids = np.array(db['ldmks']['ldmks_lids_2d']).astype(float32)
        ellipse_i = cv2.fitEllipse(iris)
        ellipse_p = cv2.fitEllipse(pupil)
        box = cv2.boundingRect(lids)

#        cv_img = cv2.imread(path)
#        cv2.ellipse(cv_img,ellipse_i,(0,255,0))
#        cv2.ellipse(cv_img,ellipse_p,(0,0,255))
#        cv2.rectangle(cv_img,(x,y),(x+w,y+h),(0,255,0),2)
#        cv2.imshow("Image", cv_img)
#        cv2.waitKey(30)
        out.write(path2+' ')
        out.write(str(ellipse_i))
        out.write(' ')
        out.write(str(ellipse_p))
        out.write(' ')
        out.write(str(box))
        out.write('\n')
cv2.destroyAllWindows()
print("Ende")