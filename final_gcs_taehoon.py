# open the room, but cannot send the messages
from src.SocGCS import SocGCS
import numpy as np
import time
import cv2 
import math

RED = (0, 0, 255)
BLUE = (255, 0, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
YELLOW =(0, 255, 255)

BGX,BGY = 1000,1000

'''

                    li = line.split()
                    lat_, lon_= li.index("lat"), li.index("lon")
                    lat, lon = int(li[lat_ + 2].split(',')[0]) , int(li[lon_+2].split(',')[0])
                    if lat !=0 and lon!= 0:
                        self.adsb_output.append((lat, lon))
'''
# NX = 800 # Node-section X-size
# NY = 400 # Node-section Y-size

resize_w, resize_h, bpp = 200, 200, 3
#img_new_w, img_new_h    = 400, 400
#img = np.zeros((resize_w, resize_h, bpp), np.uint8) # create dummy image

#rot_cen = (resize_w/2,resize_h/2)

class Gcsmain :
    def __init__(self):
        self.ANGLE = 90
        self.draw_break = False
        self.obs_loc = None
        self.tracker =[]
        self.RC_values= []
        self.s = SocGCS()
        
    def run(self):
        #socket open
       
        while self.draw_break==False: 
            msg = self.s.recv_msg()
            print("msg: ",msg)

            if msg == 'client':
                print("client connected")
        
            elif msg=='quit':
                pass
                #print("GCS socket Closed...")
                #break
        
            else: 
                # break msg into string
                replaced_msg = msg.replace("("," ").replace(")"," ").replace(","," ").replace(":"," ") #.split())
                #print("replaced_msg = ", replaced_msg)

                splited_msg = replaced_msg.split()
                #print("len(splited_msg) =", len(splited_msg),"content : ",splited_msg)

                if msg.startswith("rc"):
                    print("==Starts with RC==")
                    self.RC_values = splited_msg[1:]
                    #print("self.RC_values = ",self.RC_values)

                elif msg.startswith("cen"):
                    print("==Starts with Cen==")
                    self.obs_loc = splited_msg[1], splited_msg[2]
                    
                else :
                    print("splited_msg ==> ",splited_msg)
                    if len(splited_msg)==9:
                        newsplited_msg=[]
                        for i in range(0,6):
                            newsplited_msg.append(splited_msg[i+2])
                        self.draw_break=self.draw(newsplited_msg)

                    elif len(splited_msg)>6: #message corrupted
                        print("msg corrupted")

                    else:
                        self.draw_break= self.draw(splited_msg)
                    #dronedistance / current_gps / heading /loc_pos   
                    # for i in range(len(msg)):
                    #     print("msg[",i,"]",msg[i])
                                     
        print("break while loop")
        cv2.destroyAllWindows()
    
    def meter2pix(self,x):
        X=int(x[0]*25)
        Y=int(x[1]*25)
        return X,Y
 
    def zerocoord(self,loc):
        X = int(loc[0]+500)
        Y = int(-loc[1]+500)
        return X, Y
                  
    def draw(self,string_list):
        keyinput = cv2.waitKey(1)
        if keyinput == 13:
            print("Enter inputted, break")
            return True        

        #dronedistance / gps_lat / gps_lon / heading / loc_x / loc_y
        #string_list[0]/    [1] /   [2]     /   [3] /   [4] /   [5]
        dronedistance, gps_lat, gps_lon, str_heading, loc_x, loc_y = string_list
        int_heading = int(float(str_heading)) % 360        

        # =========make background #
        whiteboard = np.full((BGX, BGY, 3), 255, dtype=np.uint8)    # dummy WHITE board background if 255 to 0, become BLACK
        blackboard = np.zeros((BGX, BGY, bpp), np.uint8)            # dummy BLACK
        background = whiteboard
        
        #centerLoc = int(BGX/2), int(BGY/2)
        centerLoc = 0, 0
        cv2.circle(background, self.zerocoord(centerLoc), 5, RED, -1)     # draw R = 5,  filled with BLUE ==> center
        cv2.circle(background, self.zerocoord(centerLoc), BGX/4, RED, 1)  # draw R = 250, line=1, RED circle ==> barrier
        #cv2.imshow('circle2', whiteboard)

        #droneLoc = int(BGX/3), int(BGX/3) 
        #droneLoc = centerLoc[0]+int((BGX/40)*float(loc_x)), centerLoc[1]-int((BGY/40)*float(loc_y))
        #                          ->  25
        droneLoc = self.meter2pix((float(loc_x),float(loc_y)))
        cv2.circle(background, self.zerocoord(droneLoc),10,BLUE,-1) #=> draw drone
        
        # === draw drone trajectory with dot
        self.tracker.append(self.zerocoord(droneLoc))
        for i in self.tracker:
            trajactory = i[0],i[1]
            cv2.circle(background, (trajactory),5,YELLOW,-1) #=> draw drone trajacroy
       
        # === draw heading line in image ====        
        thickness = 3
        font = cv2.FONT_HERSHEY_SIMPLEX

        length = 100 # line R
        x2 = droneLoc[0] + length * math.cos(math.radians(int_heading-90))
        y2 = droneLoc[1] + length * math.sin(math.radians(int_heading-90)) 
        end_point = int(x2), int(y2)
        lined_idrone = cv2.line(background, (droneLoc), (end_point), BLACK, thickness)
        
        # === insert heading Angle as string ======
        #headingLoc = droneLoc[0]-100, droneLoc[1]+100
        headingLoc = end_point[0] , end_point[1]
        cv2.putText(lined_idrone, str_heading,  (headingLoc), font, 1, RED, 2, cv2.LINE_AA)
        #           image           txt           location of the text

        #line between drone and center
        lined_idrone = cv2.line(background, droneLoc, centerLoc, GREEN, thickness)

        #center location print
        print(self.obs_loc,type(self.obs_loc))
        obs_loc_msg = str(self.obs_loc)
        cv2.putText(lined_idrone, obs_loc_msg, (centerLoc), font, 0.5,    BLACK,  2,  cv2.LINE_AA)
        
        #print haversine distance in the middle of drone and Center
        distLoc = int((droneLoc[0]+centerLoc[0])/2), int((centerLoc[1]+droneLoc[1])/2)
        #print(dronedistance,type(dronedistance))
        cv2.putText(lined_idrone, dronedistance, (distLoc), font, 1, GREEN,  2,  cv2.LINE_AA)
        
        # == draw force direction ==
        #theta_1 : angle between x - drone
        #theta_2 : angle drone heading
        #theta_3 : angble between theta_1 and theta_2 
        #       
        theta_1 = int_heading-90 # from x-axis to drone
        theta_2 = math.degrees(math.atan2(float(loc_y), float(loc_x)))# antan2 returns y/x angle in radian -> from x-axis to drone location
        if theta_2 <0:
            theta_2 += 360
        string_theta_2 = "theta_2: "+str(int(theta_2))+"/"+str(float(loc_x))+"/"+str(float(loc_y))
        #print theta2
        cv2.putText(lined_idrone, string_theta_2, (distLoc[0]+50, distLoc[1]+50), font, 1, GREEN,  2,  cv2.LINE_AA)

        #v: vector [0,1]
        
#        print("dronedistance",len(dronedistance),dronedistance)
        
        temp_dist = dronedistance.split("'")
        if len(temp_dist)>1:
            int_dist = int(float(temp_dist[1])) 
        else:
            int_dist = int(float(temp_dist[0])) 


        print("int_dist : ",int_dist,type(int_dist))
        forced_X,forced_Y = 0,0
        
        if theta_1 <= theta_2 :
            #theta_3 = theta_3-theta_2 # from drone to force
            forced_X = 25*math.cos(math.radians(theta_1))+25*math.cos(math.radians(-theta_2))
            forced_Y = 25*math.sin(math.radians(theta_1))+25*math.sin(math.radians(-theta_2))
            
        else:
            theta_3 = theta_1-theta_2
            #theta = np.radians(theta_3)
            forced_X = 25*math.cos(math.radians(theta_1))+25*math.cos(math.radians(theta_2))
            forced_Y = 25*math.sin(math.radians(theta_1))+25*math.sin(math.radians(theta_2))
        
        print("int_dist = ",int_dist, "theta_1",theta_1,"theta_2",theta_2)
        print("forced_X/ Forced_Y", forced_X,forced_Y)
        
        forcedLoc= self.zerocoord((forced_X,forced_Y))
        #forcedLoc= int(forced_X),int(forced_Y)
        
        #r: rotation matrix []
        #r = np.array(( (np.cos(theta), -np.sin(theta)),(np.sin(theta),  np.cos(theta)) ))  
        #v = np.array((0,1))

        # rotation matrix r to v: r*v')
        #rot_v = r.dot(v)

        #r: rotation matrix []
        #x3 = droneLoc[0] + length * math.cos(math.radians(90-(theta_3)))
        #y3 = droneLoc[1] - length * math.sin(math.radians(90-(theta_3)))           
        
        length = 100 # line R
        
        direction_lined_idrone = cv2.line(lined_idrone, self.zerocoord(droneLoc), forcedLoc, GREEN, thickness)

        cv2.imshow("direction_lined_idrone",direction_lined_idrone)
        return False 
        
if __name__ == '__main__':
    gcs = Gcsmain()
    gcs.run()
    
    #msg= ['59', '37.5828811', '127.0271446','348', '-0.49468', '-0.4882']
    #temp = False
    #while temp == False: 
     #   temp = gcs.draw(msg)
