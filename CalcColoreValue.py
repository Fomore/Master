import cPickle
import cv2
import numpy as np
import fnmatch
import os

def ImageSum(Image):
    w,h,c = Image.shape
    Summe = [0 for x in range(c)]
    for i in range(w):
        for j in range(h):
            for k in range(c):
                Summe[k] = Summe[k]+Image[i,j,k]
    if c == 1:
        return Summe[0]
    else:
        return Summe 
    
def ToGray(Image, a,b,c):
    w,h,d = Image.shape
    img = np.zeros([w,h,1],dtype='uint8')
    for i in range(w):
        for j in range(h):
            img[i,j] = int(a*Image[i,j,0]+b*Image[i,j,1]+c*Image[i,j,2])
    return img

def CalcRating(Iris, Pupil):
    SumP = Pupil[0]+Pupil[1]+Pupil[2]
    SumI = Iris[0]+Iris[1]+Iris[2]
    Rating = np.array([(Iris[0]/SumI)/(Pupil[0]/SumP),(Iris[1]/SumI)/(Pupil[1]/SumP),(Iris[2]/SumI)/(Pupil[2]/SumP)])
    diff = max(Rating)/sum(Rating)
    print diff
    count = 1
    while diff < 0.5:
        count = count+1
        Rating = Rating*Rating
        diff = max(Rating)/sum(Rating)
    print (count,Rating/sum(Rating),diff)
    return Rating

AnzI =    4069880.0
GesI = np.array([147577004, 174640344, 207137983])
AnzP =   1669322.0
GesP = np.array([24481813, 30175950, 35351595])
AnzH = 98444171.0
AnzE = 5083827.0
GesH = np.array([4723734145, 6261052230, 8923742132])
GesE = np.array([370187161, 396665628, 492340673])

print ("I",GesI/AnzI)
print ("P",GesP/AnzP)
print ("E",GesE/AnzE)
print ("H",GesH/AnzH)
print ("IP",CalcRating(GesI/AnzI,GesP/AnzP))
print ("PI",CalcRating(GesP/AnzP,GesI/AnzI))
    
print ('Gray_15_15_70', 47.015512742390442/19.248461950420591) #2.44 - 0.15,0.15,0.7
print ('Gray_30_36_24', 38.106514934101249/15.522907503765001) #2.45 - 0.296,0.359,0.245
print ('Gray_60_16_24', 40.367079619055104/16.334735299720485) #2.47 - 0.59706616,  0.16214176,  0.24079208
print ('Gray_14_51_35', 44.251588498923802/18.221625306561585) #2.43 - 0.13962652,  0.51415666,  0.34621682
print ('Gray_11_59_30', 44.045516575427285/18.166344779497305) #2.42 - 0.114,0.587,0.299

Gray_P = 0
Gray_I = 0

AVGColoreH = np.array([0,0,0])
AVGColoreE = np.array([0,0,0])

GesColoreH = np.array([0,0,0])
GesColoreE = np.array([0,0,0])

out = open('workfileColore.txt', 'w+')
#(x_centre,y_centre),(minor_axis,major_axis),angle
for root, dirnames, filenames in os.walk('/home/falko/Uni/Master/Bilder/SynthEyes_data/'):
    for filename in fnmatch.filter(filenames, '*.png'):
        path = os.path.join(root, filename)
        f = open(path[:-2]+'kl', 'rb')
        db = cPickle.load(f)
        f.close()
        iris = np.array(db['ldmks']['ldmks_iris_2d']).astype(float32)
        pupil = np.array(db['ldmks']['ldmks_pupil_2d']).astype(float32)
        lids = np.array(db['ldmks']['ldmks_lids_2d']).astype(float32)

        ellipse_i = cv2.fitEllipse(iris)
        ellipse_p = cv2.fitEllipse(pupil)
        
        cv_img = cv2.imread(path)
        
        img_lids = np.zeros(cv_img.shape,dtype='uint8')
        cv2.fillConvexPoly(img_lids, lids.astype(int),(1,1,1))
        img_iris = np.zeros(cv_img.shape,dtype='uint8')
        cv2.ellipse(img_iris,ellipse_i,(1,1,1),-1)
        img_pupil = np.zeros(cv_img.shape,dtype='uint8')
        cv2.ellipse(img_pupil,ellipse_p,(1,1,1),-1)
        img_head = np.ones(cv_img.shape,dtype='uint8')
        cv2.fillConvexPoly(img_head, lids.astype(int),(0,0,0))
        
#        w,h,c = cv_img.shape        
#        img_lids = np.zeros([w,h,1],dtype='uint8')
#        cv2.fillConvexPoly(img_lids, lids.astype(int),(1))        
#        img_iris = np.zeros([w,h,1],dtype='uint8')
#        cv2.ellipse(img_iris,ellipse_i,(1),-1)
#        img_pupil = np.zeros([w,h,1],dtype='uint8')
#        cv2.ellipse(img_pupil,ellipse_p,(1),-1)  

       
#        AVGColoreI = AVGColoreI + ImageSum(cv_img*(img_iris-img_pupil)*img_lids)
#        AVGColoreP = AVGColoreP + ImageSum(cv_img*img_pupil*img_lids)
        AVGColoreH = AVGColoreH + ImageSum(cv_img*img_head)
        AVGColoreE = AVGColoreE + ImageSum(cv_img*(img_lids-(img_lids*img_iris)))
        
        GesColoreH = GesColoreH + ImageSum(img_head)
        GesColoreE = GesColoreE + ImageSum(img_lids-(img_lids*img_iris))
       
#        ImgI = (img_iris-img_pupil)*img_lids
#        ImgP = img_pupil*img_lids
#        Gray_P = Gray_P + ImageSum(ToGray(cv_img,) * ImgP)
#        Gray_I = Gray_I + ImageSum(ToGray(cv_img,0.15,0.15,0.7) * ImgI)

#        cv2.imwrite("img/"+str(count)+"Image.png", cv_img)
#        cv2.imshow("Ges", cv_img)
#        cv2.imshow("Auge", cv_img*img_head)
#        cv2.waitKey(300)
print (AVGColoreH,GesColoreH)
print (AVGColoreE,GesColoreE)
#        out.write('\n')
#cv2.destroyAllWindows()
print("Ende")